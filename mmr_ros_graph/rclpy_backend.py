"""Read the graph, parameters, topic telemetry, and /rosout in-process with rclpy."""

from collections import Counter
from collections import deque
import math
import os
import threading
import time

from mmr_ros_graph.model import Diagnostic
from mmr_ros_graph.model import Endpoint
from mmr_ros_graph.model import GraphSnapshot
from mmr_ros_graph.model import Link
from mmr_ros_graph.model import NodeInfo
from mmr_ros_graph.model import TopicInfo
from mmr_ros_graph.model import empty_snapshot
from mmr_ros_graph.model import extract_actions
from mmr_ros_graph.model import is_action_topic
from mmr_ros_graph.model import is_user_node
from mmr_ros_graph.model import join_node_name
from mmr_ros_graph.params import AsyncServiceRunner
from mmr_ros_graph.params import start_get_parameters
from mmr_ros_graph.params import start_set_parameter


LOG_LEVEL_NAMES = {
    10: 'DEBUG',
    20: 'INFO',
    30: 'WARN',
    40: 'ERROR',
    50: 'FATAL',
}


def rclpy_available():
    try:
        import rclpy  # noqa: F401
    except ImportError:
        return False
    return True


def _qos_name(value):
    return getattr(value, 'name', str(value))


def _duration_ns(duration):
    ns = int(getattr(duration, 'nanoseconds', 0) or 0)
    if ns <= 0 or ns >= 9_223_372_036_000_000_000:
        return 0
    return ns


def _compatibility(publisher_qos, subscription_qos):
    """Use Humble's own QoS check. Do not re-implement DDS matching."""
    from rclpy.qos import qos_check_compatible

    compatibility, reason = qos_check_compatible(publisher_qos, subscription_qos)
    label = compatibility.name.lower()
    text = (reason or '').strip()
    only_defaults = (
        label == 'warning'
        and 'system_default' in text.lower()
        and 'error:' not in text.lower()
    )
    if label == 'ok' or only_defaults:
        return 'ok', ''
    return label, text


def _endpoint(item):
    profile = item.qos_profile
    return Endpoint(
        join_node_name(item.node_namespace, item.node_name),
        _qos_name(profile.reliability),
        _qos_name(profile.durability),
        _qos_name(profile.history),
        int(profile.depth),
        _qos_name(getattr(profile, 'liveliness', '')),
        _duration_ns(getattr(profile, 'deadline', None)),
        _duration_ns(getattr(profile, 'lifespan', None)),
    )


def _deduped(items):
    seen = set()
    chosen = []
    for item in items:
        endpoint = _endpoint(item)
        if not is_user_node(endpoint.node):
            continue
        key = endpoint.qos_key()
        if key in seen:
            continue
        seen.add(key)
        chosen.append((endpoint, item.qos_profile))
    chosen.sort(key=lambda pair: pair[0].qos_key())
    return chosen


def json_safe(value, depth=0, max_seq=64):
    """Convert ROS message values (ordered dicts, arrays, NaN/Inf, bytes) to JSON-safe types."""
    if depth > 6:
        return '<max depth>'
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (bytes, bytearray)):
        if len(value) > max_seq:
            return f'<{len(value)} bytes>'
        return list(value)
    if hasattr(value, 'items'):
        return {str(k): json_safe(v, depth + 1, max_seq) for k, v in value.items()}
    if hasattr(value, 'tolist'):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        total = len(value)
        clipped = [json_safe(item, depth + 1, max_seq) for item in value[:max_seq]]
        if total > max_seq:
            clipped.append(f'... (+{total - max_seq} more)')
        return clipped
    return str(value)


def build_snapshot_from_raw(
    discovered_nodes,
    topic_entries,
    service_fetcher=None,
    client_fetcher=None,
    compat_fn=None,
    seq=0,
):
    """Build a GraphSnapshot from raw ROS 2 discovery records.

    Separated from rclpy Node creation so it is directly unit-testable.
    """
    node_counts = Counter()
    node_coords = {}
    for short_name, namespace in discovered_nodes:
        full_name = join_node_name(namespace, short_name)
        if not is_user_node(full_name):
            continue
        node_counts[full_name] += 1
        node_coords.setdefault(full_name, (namespace or '/', short_name))

    topics = []
    links = []
    diagnostics = []

    for topic_name, msg_type, raw_pubs, raw_subs in topic_entries:
        pubs = _deduped(raw_pubs)
        subs = _deduped(raw_subs)
        for endpoint, _profile in pubs + subs:
            if endpoint.node not in node_counts:
                node_counts[endpoint.node] = 1
                idx = endpoint.node.rfind('/')
                ns = endpoint.node[:idx] if idx > 0 else '/'
                short = endpoint.node[idx + 1:]
                node_coords.setdefault(endpoint.node, (ns, short))

        topics.append(TopicInfo(
            topic_name,
            msg_type,
            tuple(endpoint for endpoint, _profile in pubs),
            tuple(endpoint for endpoint, _profile in subs),
            action_internal=is_action_topic(topic_name),
        ))

        for pub_endpoint, pub_qos in pubs:
            for sub_endpoint, sub_qos in subs:
                if pub_endpoint.node == sub_endpoint.node:
                    continue
                if compat_fn is not None:
                    label, reason = compat_fn(pub_qos, sub_qos)
                else:
                    label, reason = ('ok', '')
                link = Link(
                    pub_endpoint.node,
                    sub_endpoint.node,
                    topic_name,
                    msg_type,
                    label,
                    reason,
                    pub_endpoint.qos_dict(),
                    sub_endpoint.qos_dict(),
                )
                links.append(link)
                if label in ('error', 'warning'):
                    kind = 'qos_mismatch' if label == 'error' else 'qos_warning'
                    diagnostics.append(Diagnostic(
                        label,
                        kind,
                        topic_name,
                        f'{pub_endpoint.node} -> {sub_endpoint.node} on {topic_name}: {reason or label}',
                    ))

    nodes = []
    for full_name in sorted(node_counts):
        ns, short = node_coords[full_name]
        count = node_counts[full_name]
        if count > 1:
            diagnostics.append(Diagnostic(
                'warning',
                'duplicate_node',
                full_name,
                f'{count} nodes share the exact name {full_name}',
            ))
        srv_pairs = ()
        cli_pairs = ()
        if service_fetcher is not None:
            try:
                srv_pairs = service_fetcher(short, ns) or ()
            except Exception:  # noqa: BLE001
                srv_pairs = ()
        if client_fetcher is not None:
            try:
                cli_pairs = client_fetcher(short, ns) or ()
            except Exception:  # noqa: BLE001
                cli_pairs = ()
        actions, services, clients = extract_actions(srv_pairs, cli_pairs)
        nodes.append(NodeInfo(
            full_name,
            ns or '/',
            short,
            count,
            services,
            clients,
            actions,
        ))

    topics.sort(key=lambda item: item.name)
    links.sort(key=lambda item: (item.topic, item.publisher, item.subscriber, item.link_id()))
    diagnostics.sort(key=lambda item: (0 if item.level == 'error' else 1, item.kind, item.target))
    return GraphSnapshot(
        tuple(nodes),
        tuple(topics),
        'rclpy',
        '',
        tuple(links),
        tuple(diagnostics),
        seq=seq,
    )


def choose_monitor_qos(pub_endpoints):
    """Choose a compatible QoS profile for monitoring a topic."""
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import HistoryPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy

    reliabilities = {ep.reliability for ep in pub_endpoints if ep.reliability}
    durabilities = {ep.durability for ep in pub_endpoints if ep.durability}

    if 'BEST_EFFORT' in reliabilities:
        reliability = ReliabilityPolicy.BEST_EFFORT
    else:
        reliability = ReliabilityPolicy.RELIABLE

    if durabilities == {'TRANSIENT_LOCAL'}:
        durability = DurabilityPolicy.TRANSIENT_LOCAL
    else:
        durability = DurabilityPolicy.VOLATILE

    return QoSProfile(
        reliability=reliability,
        durability=durability,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class RclpyReader:
    """Thread-safe background rclpy node for graph discovery, params, topic echo, and /rosout."""

    def __init__(self, poll_period=0.25, on_graph=None, on_topic=None, on_log=None):
        self._poll_period = max(0.05, float(poll_period or 0.25))
        self._on_graph = on_graph
        self._on_topic = on_topic
        self._on_log = on_log

        self._snapshot = empty_snapshot('rclpy', 'waiting for the graph')
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = None

        self._tasks = deque()
        self._task_lock = threading.Lock()

        self._logs = deque(maxlen=300)
        self._log_seq = 0
        self._graph_seq = 0
        self._watch_sub = None
        self._watch_topic = ''
        self._watch_type = ''
        self._watch_samples = deque()
        self._watch_count = 0
        self._watch_last_emit = 0.0
        self._watch_state = {'topic': '', 'type': '', 'active': False, 'hz': 0.0, 'kbps': 0.0, 'count': 0, 'sample': None, 'error': ''}

    def start(self):
        self._thread = threading.Thread(
            target=self._run, name='mmr-ros-graph', daemon=True)
        self._thread.start()
        self._ready.wait(timeout=8)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=4)

    def snapshot(self):
        with self._lock:
            return self._snapshot

    def get_logs(self, node='', min_level=0):
        with self._lock:
            items = list(self._logs)
        if node:
            norm = '/' + node.lstrip('/')
            items = [item for item in items if item['name'] == norm or item['name'] == node.lstrip('/')]
        if min_level:
            items = [item for item in items if item['level'] >= int(min_level)]
        return items

    def clear_logs(self):
        with self._lock:
            self._logs.clear()

    def topic_state(self):
        with self._lock:
            return dict(self._watch_state)

    def get_parameters(self, node_name, timeout=4.0):
        return self._enqueue_rpc(
            lambda node, runner, done: start_get_parameters(runner, node_name, done, timeout=timeout),
            timeout=timeout + 1.0,
            fallback={'node': node_name, 'parameters': [], 'error': 'Timed out waiting for parameter service'},
        )

    def set_parameter(self, node_name, name, value, param_type=None, timeout=4.0):
        return self._enqueue_rpc(
            lambda node, runner, done: start_set_parameter(
                runner, node_name, name, value, done, param_type=param_type, timeout=timeout),
            timeout=timeout + 1.0,
            fallback={'ok': False, 'reason': 'Timed out waiting for parameter service', 'node': node_name, 'name': name},
        )

    def watch_topic(self, topic_name, msg_type='', timeout=3.0):
        return self._enqueue_rpc(
            lambda node, _runner, done: done(self._configure_watch(node, topic_name, msg_type)),
            timeout=timeout,
            fallback={'topic': topic_name or '', 'active': False, 'error': 'Timed out configuring topic watch'},
        )

    def _enqueue_rpc(self, fn, timeout, fallback):
        if self._stop.is_set() or self._thread is None or not self._thread.is_alive():
            return fallback
        event = threading.Event()
        box = {}

        def done(result):
            box['result'] = result
            event.set()

        with self._task_lock:
            self._tasks.append((fn, done))
        if not event.wait(timeout=timeout):
            return fallback
        return box.get('result', fallback)

    def _store(self, snapshot):
        with self._lock:
            self._snapshot = snapshot
        self._ready.set()
        if self._on_graph is not None:
            self._on_graph(snapshot)

    def _run(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        ctx = rclpy.Context()
        rclpy.init(context=ctx)
        node_name = f'mmr_ros_graph_{os.getpid()}'
        node = rclpy.create_node(node_name, context=ctx, enable_rosout=False)
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        runner = AsyncServiceRunner(node)
        rosout_sub = self._create_rosout_sub(node)

        try:
            # Let DDS discovery resolve participant names and endpoints before emitting first snapshot.
            warmup_start = time.monotonic()
            warmup_end = warmup_start + 1.2
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.1)
                snap = self._read(node)
                elapsed = time.monotonic() - warmup_start
                if (snap.nodes and (elapsed >= 0.35 or snap.links)) or time.monotonic() >= warmup_end:
                    self._store(snap)
                    break

            last_read = time.monotonic()
            while not self._stop.is_set():
                self._drain_tasks(node, runner)
                runner.tick()
                now = time.monotonic()
                if now - last_read >= self._poll_period:
                    self._store(self._read(node))
                    last_read = now
                executor.spin_once(timeout_sec=0.05)
        finally:
            runner.cancel_all()
            self._destroy_watch(node)
            if rosout_sub is not None:
                try:
                    node.destroy_subscription(rosout_sub)
                except Exception:  # noqa: BLE001
                    pass
            executor.remove_node(node)
            node.destroy_node()
            if rclpy.ok(context=ctx):
                rclpy.shutdown(context=ctx)

    def _drain_tasks(self, node, runner):
        while True:
            with self._task_lock:
                if not self._tasks:
                    return
                fn, done = self._tasks.popleft()
            try:
                fn(node, runner, done)
            except Exception as exc:  # noqa: BLE001
                done({'error': str(exc)})

    def _create_rosout_sub(self, node):
        try:
            from rcl_interfaces.msg import Log
            from rclpy.qos import DurabilityPolicy
            from rclpy.qos import HistoryPolicy
            from rclpy.qos import QoSProfile
            from rclpy.qos import ReliabilityPolicy

            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
            )
            return node.create_subscription(Log, '/rosout', self._on_rosout_msg, qos)
        except Exception:  # noqa: BLE001
            return None

    def _on_rosout_msg(self, msg):
        raw_name = str(getattr(msg, 'name', '') or '')
        norm_name = '/' + raw_name.lstrip('/') if raw_name else ''
        if not is_user_node(norm_name):
            return
        stamp = getattr(msg, 'stamp', None)
        sec = int(getattr(stamp, 'sec', 0) or 0)
        nanosec = int(getattr(stamp, 'nanosec', 0) or 0)
        level = int(getattr(msg, 'level', 20) or 20)
        with self._lock:
            self._log_seq += 1
            entry = {
                'seq': self._log_seq,
                'stamp': sec + nanosec / 1_000_000_000.0,
                'level': level,
                'level_name': LOG_LEVEL_NAMES.get(level, str(level)),
                'name': norm_name,
                'msg': str(getattr(msg, 'msg', '') or ''),
                'file': str(getattr(msg, 'file', '') or ''),
                'function': str(getattr(msg, 'function', '') or ''),
                'line': int(getattr(msg, 'line', 0) or 0),
            }
            self._logs.append(entry)
        if self._on_log is not None:
            self._on_log(entry)

    def _destroy_watch(self, node):
        if self._watch_sub is not None:
            try:
                node.destroy_subscription(self._watch_sub)
            except Exception:  # noqa: BLE001
                pass
            self._watch_sub = None

    def _configure_watch(self, node, topic_name, msg_type):
        self._destroy_watch(node)
        self._watch_samples.clear()
        self._watch_count = 0
        self._watch_last_emit = 0.0

        clean_topic = (topic_name or '').strip()
        if not clean_topic:
            self._watch_topic = ''
            self._watch_type = ''
            state = {'topic': '', 'type': '', 'active': False, 'hz': 0.0, 'kbps': 0.0, 'count': 0, 'sample': None, 'error': ''}
            with self._lock:
                self._watch_state = state
            if self._on_topic is not None:
                self._on_topic(state)
            return state

        snapshot = self.snapshot()
        topic_info = next((item for item in snapshot.topics if item.name == clean_topic), None)
        resolved_type = msg_type or (topic_info.msg_type if topic_info else '')
        if not resolved_type or resolved_type == 'unknown':
            for name, types in node.get_topic_names_and_types():
                if name == clean_topic and types:
                    resolved_type = types[0]
                    break
        if not resolved_type or resolved_type == 'unknown':
            state = {'topic': clean_topic, 'type': 'unknown', 'active': False, 'hz': 0.0, 'kbps': 0.0, 'count': 0, 'sample': None, 'error': 'Unknown topic type'}
            with self._lock:
                self._watch_state = state
            return state

        try:
            from rosidl_runtime_py.utilities import get_message
            msg_cls = get_message(resolved_type)
        except Exception as exc:  # noqa: BLE001
            state = {'topic': clean_topic, 'type': resolved_type, 'active': False, 'hz': 0.0, 'kbps': 0.0, 'count': 0, 'sample': None, 'error': f'Cannot load message type {resolved_type}: {exc}'}
            with self._lock:
                self._watch_state = state
            return state

        pub_endpoints = topic_info.publishers if topic_info else ()
        qos = choose_monitor_qos(pub_endpoints)
        self._watch_topic = clean_topic
        self._watch_type = resolved_type
        self._watch_sub = node.create_subscription(
            msg_cls,
            clean_topic,
            lambda msg, t=clean_topic, mt=resolved_type: self._on_watch_msg(t, mt, msg),
            qos,
        )
        state = {'topic': clean_topic, 'type': resolved_type, 'active': True, 'hz': 0.0, 'kbps': 0.0, 'count': 0, 'sample': None, 'error': ''}
        with self._lock:
            self._watch_state = state
        if self._on_topic is not None:
            self._on_topic(state)
        return state

    def _on_watch_msg(self, topic_name, msg_type, msg):
        if topic_name != self._watch_topic:
            return
        now = time.monotonic()
        byte_len = 0
        try:
            from rclpy.serialization import serialize_message
            byte_len = len(serialize_message(msg))
        except Exception:  # noqa: BLE001
            byte_len = 0

        self._watch_count += 1
        self._watch_samples.append((now, byte_len))
        cutoff = now - 2.0
        while self._watch_samples and self._watch_samples[0][0] < cutoff:
            self._watch_samples.popleft()

        if now - self._watch_last_emit < 0.2 and self._watch_count > 1:
            return
        self._watch_last_emit = now

        n = len(self._watch_samples)
        if n >= 2:
            span = max(0.05, self._watch_samples[-1][0] - self._watch_samples[0][0])
            hz = round((n - 1) / span, 2)
            total_bytes = sum(size for _ts, size in self._watch_samples)
            kbps = round((total_bytes / span) / 1024.0, 2)
        else:
            hz = 0.0
            kbps = 0.0

        try:
            from rosidl_runtime_py.convert import message_to_ordereddict
            sample = json_safe(message_to_ordereddict(msg))
        except Exception as exc:  # noqa: BLE001
            sample = {'_repr': str(msg), '_warning': str(exc)}

        state = {
            'topic': topic_name,
            'type': msg_type,
            'active': True,
            'hz': hz,
            'kbps': kbps,
            'count': self._watch_count,
            'sample': sample,
            'error': '',
        }
        with self._lock:
            self._watch_state = state
        if self._on_topic is not None:
            self._on_topic(state)

    def _read(self, node):
        try:
            discovered = node.get_node_names_and_namespaces()
            topic_entries = []
            for topic_name, types in node.get_topic_names_and_types():
                msg_type = types[0] if types else 'unknown'
                raw_pubs = node.get_publishers_info_by_topic(topic_name)
                raw_subs = node.get_subscriptions_info_by_topic(topic_name)
                topic_entries.append((topic_name, msg_type, raw_pubs, raw_subs))
            return build_snapshot_from_raw(
                discovered,
                topic_entries,
                service_fetcher=lambda short, ns: node.get_service_names_and_types_by_node(short, ns),
                client_fetcher=lambda short, ns: node.get_client_names_and_types_by_node(short, ns),
                compat_fn=_compatibility,
                seq=0,
            )
        except Exception as exc:  # noqa: BLE001
            return empty_snapshot('rclpy', str(exc))
