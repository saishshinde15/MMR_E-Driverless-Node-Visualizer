"""Read the graph by shelling out to the ros2 command.

Used when this process is not inside a sourced ROS environment.
That is the macOS terminal, where `ros2` is a wrapper, and any
machine where rclpy cannot be imported.
"""

import shutil
import subprocess

from mmr_ros_graph.model import Endpoint
from mmr_ros_graph.model import GraphSnapshot
from mmr_ros_graph.model import TopicInfo
from mmr_ros_graph.model import empty_snapshot
from mmr_ros_graph.model import is_user_node
from mmr_ros_graph.model import join_node_name
from mmr_ros_graph.model import unique_endpoints


def ros2_executable():
    return shutil.which('ros2')


def _run(args, timeout):
    completed = subprocess.run(
        args,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout or '').strip()
        raise RuntimeError(detail or 'ros2 command failed')
    return completed.stdout


def parse_topic_list(text):
    """Parse `ros2 topic list -t` into (name, type) pairs."""
    found = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.lower().startswith('warning'):
            continue
        if line.endswith(']') and ' [' in line:
            name, typed = line.rsplit(' [', 1)
            found.append((name.strip(), typed[:-1].strip()))
        elif line.startswith('/'):
            found.append((line, ''))
    return found


def parse_node_list(text):
    nodes = []
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('/'):
            nodes.append(line)
    return tuple(nodes)


def _blank_endpoint():
    return {
        'name': '',
        'namespace': '/',
        'kind': '',
        'reliability': '',
        'durability': '',
        'history': '',
        'depth': 0,
    }


def _as_endpoint(current):
    return Endpoint(
        join_node_name(current['namespace'], current['name']),
        current['reliability'],
        current['durability'],
        current['history'],
        current['depth'],
    )


def parse_topic_info(text, topic_name, fallback_type):
    """Parse `ros2 topic info <name> -v`, including the QoS of each end."""
    msg_type = fallback_type
    publishers = []
    subscribers = []
    current = _blank_endpoint()

    def flush():
        if not current['name'] or not current['kind']:
            return
        endpoint = _as_endpoint(current)
        if not endpoint.node:
            return
        if current['kind'] == 'PUBLISHER':
            publishers.append(endpoint)
        elif current['kind'] == 'SUBSCRIPTION':
            subscribers.append(endpoint)

    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('Type:') and not msg_type:
            msg_type = line.split(':', 1)[1].strip()
        elif line.startswith('Node name:'):
            flush()
            current = _blank_endpoint()
            current['name'] = line.split(':', 1)[1].strip()
        elif line.startswith('Node namespace:'):
            current['namespace'] = line.split(':', 1)[1].strip()
        elif line.startswith('Endpoint type:'):
            current['kind'] = line.split(':', 1)[1].strip().upper()
        elif line.startswith('Reliability:'):
            current['reliability'] = line.split(':', 1)[1].strip()
        elif line.startswith('Durability:'):
            current['durability'] = line.split(':', 1)[1].strip()
        elif line.startswith('History'):
            value = line.split(':', 1)[1].strip()
            if '(' in value and value.endswith(')'):
                history, depth = value.split('(', 1)
                current['history'] = history.strip()
                current['depth'] = int(depth[:-1].strip() or 0)
            else:
                current['history'] = value
    flush()
    return TopicInfo(
        topic_name,
        msg_type or 'unknown',
        unique_endpoints(publishers),
        unique_endpoints(subscribers),
    )


def _nodes_from_topics(topics, listed_nodes):
    names = set(listed_nodes)
    for topic in topics:
        names.update(item.node for item in topic.publishers)
        names.update(item.node for item in topic.subscribers)
    return tuple(sorted(name for name in names if is_user_node(name)))


def read_graph_cli(timeout=25):
    """One reading through several ros2 commands."""
    program = ros2_executable()
    if not program:
        return empty_snapshot('cli', 'ros2 was not found on PATH')
    try:
        listed_nodes = parse_node_list(_run([program, 'node', 'list'], timeout))
        listed_topics = parse_topic_list(
            _run([program, 'topic', 'list', '-t'], timeout))
        topics = []
        for name, msg_type in listed_topics:
            info = _run([program, 'topic', 'info', name, '-v'], timeout)
            topics.append(parse_topic_info(info, name, msg_type))
        topic_tuple = tuple(topics)
        return GraphSnapshot(
            _nodes_from_topics(topic_tuple, listed_nodes),
            topic_tuple,
            'cli',
        )
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        return empty_snapshot('cli', str(exc))



