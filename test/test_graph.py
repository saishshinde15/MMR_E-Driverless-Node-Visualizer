import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mmr_ros_graph.cli_backend import parse_node_list
from mmr_ros_graph.cli_backend import parse_topic_info
from mmr_ros_graph.cli_backend import parse_topic_list
from mmr_ros_graph.model import Diagnostic
from mmr_ros_graph.model import Endpoint
from mmr_ros_graph.model import GraphSnapshot
from mmr_ros_graph.model import Link
from mmr_ros_graph.model import NodeInfo
from mmr_ros_graph.model import TopicInfo
from mmr_ros_graph.model import extract_actions
from mmr_ros_graph.model import is_user_node
from mmr_ros_graph.model import join_node_name
from mmr_ros_graph.model import qos_mismatch
from mmr_ros_graph.model import snapshot_from_payload
from mmr_ros_graph.model import split_node_name


INFO = """
Type: turtlesim/msg/Pose

Publisher count: 1

Node name: turtlesim
Node namespace: /
Topic type: turtlesim/msg/Pose
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: VOLATILE

Subscription count: 1

Node name: teleop
Node namespace: /team
Topic type: turtlesim/msg/Pose
Endpoint type: SUBSCRIPTION
"""


class GraphTests(unittest.TestCase):
    def test_join_and_split_node_name(self):
        self.assertEqual(join_node_name('/', 'turtlesim'), '/turtlesim')
        self.assertEqual(join_node_name('/team', 'teleop'), '/team/teleop')
        self.assertEqual(join_node_name('/team', '/already'), '/already')
        self.assertEqual(split_node_name('/turtlesim'), ('/', 'turtlesim'))
        self.assertEqual(split_node_name('/team/teleop'), ('/team', 'teleop'))

    def test_is_user_node_filters_viewer_and_cli(self):
        self.assertTrue(is_user_node('/talker'))
        self.assertFalse(is_user_node(''))
        self.assertFalse(is_user_node('/_NODE_NAME_UNKNOWN_'))
        self.assertFalse(is_user_node('_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_'))
        self.assertFalse(is_user_node('/mmr_ros_graph'))
        self.assertFalse(is_user_node('/mmr_ros_graph_12345'))
        self.assertFalse(is_user_node('/_ros2cli_daemon_0_abc'))

    def test_parse_lists_and_info(self):
        listed = parse_topic_list(
            '/turtle1/pose [turtlesim/msg/Pose]\n/chatter [std_msgs/msg/String]\n')
        self.assertEqual(listed[0], ('/turtle1/pose', 'turtlesim/msg/Pose'))
        info = parse_topic_info(INFO, '/turtle1/pose', 'turtlesim/msg/Pose')
        self.assertEqual(info.publishers[0].node, '/turtlesim')
        self.assertEqual(info.publishers[0].reliability, 'RELIABLE')
        self.assertEqual(info.publishers[0].depth, 10)
        self.assertEqual(info.subscribers[0].node, '/team/teleop')
        self.assertEqual(
            parse_node_list('\n/turtlesim\nnot a node\n/teleop\n'),
            ('/turtlesim', '/teleop'))

    def test_extract_actions(self):
        services = [
            ('/fibonacci/_action/send_goal', ['action_tutorials_interfaces/action/Fibonacci_SendGoal']),
            ('/fibonacci/_action/cancel_goal', ['action_msgs/srv/CancelGoal']),
            ('/fibonacci/_action/get_result', ['action_tutorials_interfaces/action/Fibonacci_GetResult']),
            ('/add_two_ints', ['example_interfaces/srv/AddTwoInts']),
            ('/server/list_parameters', ['rcl_interfaces/srv/ListParameters']),
        ]
        clients = [
            ('/navigate/_action/send_goal', ['nav2_msgs/action/NavigateToPose_SendGoal']),
        ]
        actions, plain_srvs, plain_clis = extract_actions(services, clients)
        self.assertEqual(len(actions), 2)
        self.assertEqual(actions[0].name, '/fibonacci')
        self.assertEqual(actions[0].action_type, 'action_tutorials_interfaces/action/Fibonacci')
        self.assertEqual(actions[0].role, 'server')
        self.assertEqual(actions[1].name, '/navigate')
        self.assertEqual(actions[1].role, 'client')
        self.assertEqual([s.name for s in plain_srvs], ['/add_two_ints', '/server/list_parameters'])
        self.assertEqual(plain_srvs[0].msg_type, 'example_interfaces/srv/AddTwoInts')
        self.assertFalse(plain_srvs[0].system)
        self.assertTrue(plain_srvs[1].system)
        self.assertEqual(plain_clis, ())

    def test_snapshot_dict_shape_and_roundtrip(self):
        topic = TopicInfo(
            '/chatter', 'std_msgs/msg/String',
            (Endpoint('/talker', 'RELIABLE', 'VOLATILE', 'KEEP_LAST', 10, 'AUTOMATIC', 50_000_000),),
            (Endpoint('/listener', 'RELIABLE', 'TRANSIENT_LOCAL', 'KEEP_LAST', 10),))
        node_a = NodeInfo('/talker', '/', 'talker', 1)
        node_b = NodeInfo('/listener', '/', 'listener', 2)
        link = Link(
            '/talker', '/listener', '/chatter', 'std_msgs/msg/String',
            'error', 'durability mismatch',
            topic.publishers[0].qos_dict(), topic.subscribers[0].qos_dict(),
        )
        diag = Diagnostic('warning', 'duplicate_node', '/listener', '2 nodes share /listener')
        payload = GraphSnapshot((node_a, node_b), (topic,), 'test', '', (link,), (diag,)).to_dict()
        text = json.dumps(payload)
        rebuilt = snapshot_from_payload(json.loads(text))

        self.assertEqual(payload['topics'][0]['publishers'][0]['node'], '/talker')
        self.assertFalse(payload['topics'][0]['system'])
        self.assertEqual(rebuilt.nodes[1].count, 2)
        self.assertEqual(rebuilt.links[0].compatibility, 'error')
        self.assertEqual(rebuilt.diagnostics[0].kind, 'duplicate_node')
        self.assertEqual(rebuilt.topics[0].publishers[0].deadline_ns, 50_000_000)
        reasons = qos_mismatch(topic.publishers[0], topic.subscribers[0])
        self.assertIn('TRANSIENT_LOCAL', reasons[0])


if __name__ == '__main__':
    unittest.main()
