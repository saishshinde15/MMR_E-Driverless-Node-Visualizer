import io
import json
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mmr_ros_graph.model import Endpoint
from mmr_ros_graph.model import GraphSnapshot
from mmr_ros_graph.model import TopicInfo
from mmr_ros_graph.params import AsyncServiceRunner
from mmr_ros_graph.params import build_parameter_value
from mmr_ros_graph.params import decode_parameter_value
from mmr_ros_graph.params import merge_parameter_details
from mmr_ros_graph.params import normalize_node_name
from mmr_ros_graph.params import start_get_parameters
from mmr_ros_graph.params import start_set_parameter
from mmr_ros_graph.rclpy_backend import build_snapshot_from_raw
from mmr_ros_graph.rclpy_backend import json_safe
from mmr_ros_graph.reader import DiscoveryStreamReader
from mmr_ros_graph.server import Hub
from mmr_ros_graph.server import make_handler


def _install_fake_rcl_interfaces():
    """Provide lightweight rcl_interfaces message/service stubs when testing outside ROS."""
    if 'rcl_interfaces.msg' in sys.modules and 'rcl_interfaces.srv' in sys.modules:
        return

    pkg = types.ModuleType('rcl_interfaces')
    msg_mod = types.ModuleType('rcl_interfaces.msg')
    srv_mod = types.ModuleType('rcl_interfaces.srv')

    class ParameterValue:
        def __init__(self):
            self.type = 0
            self.bool_value = False
            self.integer_value = 0
            self.double_value = 0.0
            self.string_value = ''
            self.byte_array_value = []
            self.bool_array_value = []
            self.integer_array_value = []
            self.double_array_value = []
            self.string_array_value = []

    class Parameter:
        def __init__(self, name='', value=None):
            self.name = name
            self.value = value or ParameterValue()

    class _ReqBase:
        def __init__(self, **kwargs):
            for k, v in kwargs.items():
                setattr(self, k, v)

    class ListParameters:
        class Request(_ReqBase):
            pass

    class DescribeParameters:
        class Request(_ReqBase):
            pass

    class GetParameters:
        class Request(_ReqBase):
            pass

    class GetParameterTypes:
        class Request(_ReqBase):
            pass

    class SetParameters:
        class Request(_ReqBase):
            pass

    msg_mod.ParameterValue = ParameterValue
    msg_mod.Parameter = Parameter
    srv_mod.ListParameters = ListParameters
    srv_mod.DescribeParameters = DescribeParameters
    srv_mod.GetParameters = GetParameters
    srv_mod.GetParameterTypes = GetParameterTypes
    srv_mod.SetParameters = SetParameters
    pkg.msg = msg_mod
    pkg.srv = srv_mod

    sys.modules['rcl_interfaces'] = pkg
    sys.modules['rcl_interfaces.msg'] = msg_mod
    sys.modules['rcl_interfaces.srv'] = srv_mod


class _FakeDuration:
    def __init__(self, ns=0):
        self.nanoseconds = ns


class _FakeQos:
    def __init__(self, reliability='RELIABLE', durability='VOLATILE', history='KEEP_LAST', depth=10, deadline_ns=0):
        self.reliability = SimpleNamespace(name=reliability)
        self.durability = SimpleNamespace(name=durability)
        self.history = SimpleNamespace(name=history)
        self.depth = depth
        self.liveliness = SimpleNamespace(name='AUTOMATIC')
        self.deadline = _FakeDuration(deadline_ns)
        self.lifespan = _FakeDuration(0)


class _FakeTopicEndpoint:
    def __init__(self, node_name, node_namespace='/', qos=None):
        self.node_name = node_name
        self.node_namespace = node_namespace
        self.qos_profile = qos or _FakeQos()


class RclpyBackendUnitTests(unittest.TestCase):
    def test_build_snapshot_detects_duplicates_actions_and_multi_qos_links(self):
        discovered = [
            ('talker', '/'),
            ('talker', '/'),
            ('listener', '/team'),
            ('mmr_ros_graph_999', '/'),
        ]
        pub_rel = _FakeTopicEndpoint('talker', '/', _FakeQos('RELIABLE', 'VOLATILE'))
        pub_be = _FakeTopicEndpoint('talker', '/', _FakeQos('BEST_EFFORT', 'VOLATILE'))
        sub_rel = _FakeTopicEndpoint('listener', '/team', _FakeQos('RELIABLE', 'TRANSIENT_LOCAL'))

        topics = [
            ('/chatter', 'std_msgs/msg/String', [pub_rel, pub_be], [sub_rel]),
            ('/drive/_action/feedback', 'nav_msgs/msg/Path', [pub_rel], []),
        ]

        def srv_fetch(short, ns):
            if short == 'talker' and ns == '/':
                return [
                    ('/drive/_action/send_goal', 'pkg/action/Drive_SendGoal'),
                    ('/talker/list_parameters', 'rcl_interfaces/srv/ListParameters'),
                    ('/reset', 'std_srvs/srv/Trigger'),
                ]
            return []

        def compat_fn(pq, sq):
            if pq.reliability.name == 'BEST_EFFORT' and sq.reliability.name == 'RELIABLE':
                return 'error', 'reliability mismatch'
            return 'warning', 'durability mismatch'

        snap = build_snapshot_from_raw(
            discovered,
            topics,
            service_fetcher=srv_fetch,
            client_fetcher=lambda _s, _n: [],
            compat_fn=compat_fn,
        )

        self.assertEqual([n.id for n in snap.nodes], ['/talker', '/team/listener'])
        talker = snap.nodes[0]
        self.assertEqual(talker.count, 2)
        self.assertEqual(len(talker.actions), 1)
        self.assertEqual(talker.actions[0].name, '/drive')
        self.assertEqual([s.name for s in talker.services if not s.system], ['/reset'])

        # Two publishers on /talker with different QoS produce 2 distinct link IDs
        self.assertEqual(len(snap.links), 2)
        self.assertNotEqual(snap.links[0].link_id(), snap.links[1].link_id())
        diag_kinds = {d.kind for d in snap.diagnostics}
        self.assertIn('duplicate_node', diag_kinds)
        self.assertIn('qos_mismatch', diag_kinds)
        self.assertIn('qos_warning', diag_kinds)
        self.assertTrue(snap.topics[1].is_action_internal())

    def test_json_safe_handles_nan_bytes_and_truncation(self):
        payload = json_safe({
            'nan': float('nan'),
            'inf': float('inf'),
            'raw': b'\x01\x02' * 50,
            'arr': list(range(100)),
        }, max_seq=5)
        dumped = json.dumps(payload)
        parsed = json.loads(dumped)
        self.assertIsNone(parsed['nan'])
        self.assertIsNone(parsed['inf'])
        self.assertEqual(parsed['raw'], '<100 bytes>')
        self.assertEqual(len(parsed['arr']), 6)
        self.assertIn('more', parsed['arr'][-1])


class ParamsUnitTests(unittest.TestCase):
    def setUp(self):
        _install_fake_rcl_interfaces()

    def test_value_coercion_and_decode(self):
        self.assertEqual(normalize_node_name('talker'), '/talker')
        pv, tname, val = build_parameter_value('bool', 'true')
        self.assertEqual((tname, val), ('bool', True))
        self.assertEqual(decode_parameter_value(pv), ('bool', True))

        pv_arr, tname_arr, val_arr = build_parameter_value('int_array', '[1, 2, 3]')
        self.assertEqual((tname_arr, val_arr), ('int_array', [1, 2, 3]))
        self.assertEqual(decode_parameter_value(pv_arr), ('int_array', [1, 2, 3]))

        with self.assertRaises(ValueError):
            build_parameter_value('int', True)

    def test_async_get_and_set_parameters(self):
        from rcl_interfaces.msg import ParameterValue

        pv = ParameterValue()
        pv.type = 2
        pv.integer_value = 42

        desc = SimpleNamespace(
            name='rate_hz',
            type=2,
            description='Publish rate',
            additional_constraints='',
            read_only=False,
            dynamic_typing=False,
            integer_range=[SimpleNamespace(from_value=1, to_value=100, step=1)],
            floating_point_range=[],
        )

        class _FakeFuture:
            def __init__(self, result):
                self._result = result
                self._cbs = []

            def done(self):
                return True

            def result(self):
                return self._result

            def add_done_callback(self, cb):
                self._cbs.append(cb)
                cb(self)

        class _FakeClient:
            def __init__(self, srv_name):
                self.srv_name = srv_name

            def service_is_ready(self):
                return True

            def call_async(self, req):
                if self.srv_name.endswith('/list_parameters'):
                    return _FakeFuture(SimpleNamespace(result=SimpleNamespace(names=['rate_hz'])))
                if self.srv_name.endswith('/describe_parameters'):
                    return _FakeFuture(SimpleNamespace(descriptors=[desc]))
                if self.srv_name.endswith('/get_parameters'):
                    return _FakeFuture(SimpleNamespace(values=[pv]))
                if self.srv_name.endswith('/get_parameter_types'):
                    return _FakeFuture(SimpleNamespace(types=[2]))
                if self.srv_name.endswith('/set_parameters'):
                    return _FakeFuture(SimpleNamespace(results=[SimpleNamespace(successful=True, reason='')]))
                raise AssertionError(f'Unexpected service {self.srv_name}')

        class _FakeNode:
            def __init__(self):
                self.destroyed = 0

            def create_client(self, _srv_type, srv_name):
                return _FakeClient(srv_name)

            def destroy_client(self, _client):
                self.destroyed += 1

        fake_node = _FakeNode()
        runner = AsyncServiceRunner(fake_node)

        got = {}
        start_get_parameters(runner, '/talker', got.update)
        self.assertEqual(got['error'], '')
        self.assertEqual(got['parameters'][0]['name'], 'rate_hz')
        self.assertEqual(got['parameters'][0]['value'], 42)
        self.assertEqual(got['parameters'][0]['integer_range']['to'], 100)

        set_res = {}
        start_set_parameter(runner, '/talker', 'rate_hz', '55', set_res.update)
        self.assertTrue(set_res['ok'])
        self.assertEqual(set_res['value'], 55)
        self.assertEqual(fake_node.destroyed, 5)

    def test_merge_parameter_details(self):
        _install_fake_rcl_interfaces()
        from rcl_interfaces.msg import ParameterValue

        pv = ParameterValue()
        pv.type = 4
        pv.string_value = 'fast'
        desc = SimpleNamespace(name='mode', type=4, description='Drive mode', read_only=True)
        merged = merge_parameter_details(['mode'], [desc], [pv])
        self.assertEqual(merged[0]['value'], 'fast')
        self.assertTrue(merged[0]['read_only'])


class BridgeAndServerTests(unittest.TestCase):
    def test_discovery_stream_reader_rpc_and_clean_eof_exit(self):
        worker_script = (
            'import json, sys\n'
            'print(json.dumps({"kind": "graph", "payload": {"nodes": [{"id": "/talker"}], "topics": [], "links": [], "source": "rclpy", "error": ""}}), flush=True)\n'
            'for line in sys.stdin:\n'
            '    req = json.loads(line)\n'
            '    if req["op"] == "get_params":\n'
            '        print(json.dumps({"kind": "reply", "id": req["id"], "payload": {"node": req["node"], "parameters": [{"name": "use_sim_time", "type": "bool", "value": False}], "error": ""}}), flush=True)\n'
            '    elif req["op"] == "set_param":\n'
            '        print(json.dumps({"kind": "reply", "id": req["id"], "payload": {"ok": True, "reason": "", "value": req["value"]}}), flush=True)\n'
            '    elif req["op"] == "watch_topic":\n'
            '        print(json.dumps({"kind": "topic_sample", "payload": {"topic": req["topic"], "active": True, "hz": 2.0, "kbps": 0.1, "count": 1, "sample": {"data": "hi"}}}), flush=True)\n'
            '        print(json.dumps({"kind": "reply", "id": req["id"], "payload": {"topic": req["topic"], "active": True, "error": ""}}), flush=True)\n'
        )
        reader = DiscoveryStreamReader(command=[sys.executable, '-c', worker_script])
        reader.start()
        try:
            deadline = time.monotonic() + 3.0
            while not reader.snapshot().nodes and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertEqual(reader.snapshot().nodes[0].id, '/talker')

            params = reader.get_parameters('/talker')
            self.assertEqual(params['parameters'][0]['name'], 'use_sim_time')

            res = reader.set_parameter('/talker', 'use_sim_time', True, param_type='bool')
            self.assertTrue(res['ok'])

            watch = reader.watch_topic('/chatter')
            self.assertTrue(watch['active'])
            self.assertEqual(reader.topic_state()['hz'], 2.0)
        finally:
            reader.stop()

    def test_discovery_stream_reader_captures_stderr_on_crash(self):
        crash_script = 'import sys; print("ros2 boom", file=sys.stderr); sys.exit(2)'
        reader = DiscoveryStreamReader(command=[sys.executable, '-c', crash_script])
        reader.start()
        try:
            deadline = time.monotonic() + 2.0
            while 'boom' not in reader.snapshot().error and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertIn('ros2 boom', reader.snapshot().error)
        finally:
            reader.stop()

    def test_http_handler_routes_and_sse_shutdown(self):
        class _FakeReader:
            def get_parameters(self, node):
                return {'node': node, 'parameters': [{'name': 'gain', 'type': 'double', 'value': 1.5}], 'error': ''}

            def set_parameter(self, node, name, value, param_type=None):
                return {'ok': True, 'reason': '', 'node': node, 'name': name, 'type': param_type or 'double', 'value': value}

            def watch_topic(self, topic, msg_type=''):
                return {'topic': topic, 'type': msg_type, 'active': bool(topic), 'hz': 0.0, 'kbps': 0.0, 'count': 0, 'sample': None, 'error': ''}

            def topic_state(self):
                return {'topic': '', 'active': False}

            def get_logs(self, node='', min_level=0):
                return [{'seq': 1, 'level': 30, 'level_name': 'WARN', 'name': '/talker', 'msg': 'hello'}]

            def clear_logs(self):
                return None

        hub = Hub()
        snap = GraphSnapshot(
            ('/talker',),
            (TopicInfo('/chatter', 'std_msgs/msg/String', (Endpoint('/talker'),), ()),),
            'test',
        )
        hub.publish(snap)
        handler_cls = make_handler(hub, reader=_FakeReader())

        def run_request(raw_http):
            req_buf = io.BytesIO(raw_http.encode('utf-8'))
            resp_buf = io.BytesIO()

            class _Sock:
                def makefile(self, mode, *_args):
                    return req_buf if 'r' in mode else resp_buf

                def sendall(self, data):
                    resp_buf.write(data)

            handler_cls(_Sock(), ('127.0.0.1', 12345), SimpleNamespace())
            raw_resp = resp_buf.getvalue().decode('utf-8')
            head, body = raw_resp.split('\r\n\r\n', 1)
            status_line = head.splitlines()[0]
            return status_line, head, body

        status, _h, body = run_request('GET / HTTP/1.1\r\nHost: localhost\r\n\r\n')
        self.assertIn('200 OK', status)
        self.assertIn('MMR', body)

        status, _h, body = run_request('GET /api/graph HTTP/1.1\r\nHost: localhost\r\n\r\n')
        self.assertIn('200 OK', status)
        self.assertEqual(json.loads(body)['nodes'][0]['id'], '/talker')

        status, head, _b = run_request('GET /api/export HTTP/1.1\r\nHost: localhost\r\n\r\n')
        self.assertIn('200 OK', status)
        self.assertIn('attachment', head)

        status, _h, body = run_request('GET /api/params?node=/talker HTTP/1.1\r\nHost: localhost\r\n\r\n')
        self.assertIn('200 OK', status)
        self.assertEqual(json.loads(body)['parameters'][0]['name'], 'gain')

        post_body = json.dumps({'node': '/talker', 'name': 'gain', 'value': 2.5})
        status, _h, body = run_request(
            f'POST /api/params HTTP/1.1\r\nHost: localhost\r\nContent-Length: {len(post_body)}\r\n\r\n{post_body}')
        self.assertIn('200 OK', status)
        self.assertTrue(json.loads(body)['ok'])

        bad_body = json.dumps({'node': '/talker'})
        status, _h, _b = run_request(
            f'POST /api/params HTTP/1.1\r\nHost: localhost\r\nContent-Length: {len(bad_body)}\r\n\r\n{bad_body}')
        self.assertIn('400', status)

        watch_body = json.dumps({'topic': '/chatter'})
        status, _h, body = run_request(
            f'POST /api/topic/watch HTTP/1.1\r\nHost: localhost\r\nContent-Length: {len(watch_body)}\r\n\r\n{watch_body}')
        self.assertIn('200 OK', status)
        self.assertTrue(json.loads(body)['active'])

        status, _h, body = run_request('GET /api/logs HTTP/1.1\r\nHost: localhost\r\n\r\n')
        self.assertIn('200 OK', status)
        self.assertEqual(json.loads(body)['logs'][0]['level_name'], 'WARN')

        # Verify SSE stream emits initial graph, topic/log events, and wakes immediately on hub.close()
        sse_out = io.BytesIO()
        sse_req = io.BytesIO(b'GET /api/stream HTTP/1.1\r\nHost: localhost\r\n\r\n')

        class _SseSock:
            def makefile(self, mode, *_args):
                return sse_req if 'r' in mode else sse_out

            def sendall(self, data):
                sse_out.write(data)

        sse_thread = threading.Thread(
            target=lambda: handler_cls(_SseSock(), ('127.0.0.1', 12346), SimpleNamespace()),
            daemon=True,
        )
        sse_thread.start()
        time.sleep(0.05)
        hub.publish_topic({'topic': '/chatter', 'active': True, 'hz': 5.0})
        hub.publish_log({'seq': 2, 'level': 30, 'name': '/talker', 'msg': 'warn msg'})
        time.sleep(0.05)
        t0 = time.monotonic()
        hub.close()
        sse_thread.join(timeout=1.0)
        self.assertFalse(sse_thread.is_alive())
        self.assertLess(time.monotonic() - t0, 1.0)
        stream_text = sse_out.getvalue().decode('utf-8')
        self.assertIn('data: ', stream_text)
        self.assertIn('event: topic', stream_text)
        self.assertIn('event: log', stream_text)


if __name__ == '__main__':
    unittest.main()
