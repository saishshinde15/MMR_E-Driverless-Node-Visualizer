"""Open the live graph page, stream snapshots, or run the ROS 2 worker."""

import argparse
import json
import socket
import sys
import threading
import time
import webbrowser

from mmr_ros_graph.model import empty_snapshot
from mmr_ros_graph.rclpy_backend import RclpyReader
from mmr_ros_graph.rclpy_backend import rclpy_available
from mmr_ros_graph.reader import start_reader
from mmr_ros_graph.server import Hub
from mmr_ros_graph.server import serve


PENDING_ERRORS = ('waiting for the graph', 'waiting for discovery')


def _free_port(host, preferred):
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind((host, preferred))
        return preferred
    except OSError:
        probe.close()
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.bind((host, 0))
        return probe.getsockname()[1]
    finally:
        probe.close()


def _watch_stdin_eof(stop):
    """Exit worker/stream inside containers when the parent host pipe closes."""
    try:
        while not stop.is_set():
            chunk = sys.stdin.read(1)
            if chunk == '':
                stop.set()
                break
    except (OSError, ValueError):
        stop.set()


def _emit_json_line(lock, obj):
    text = json.dumps(obj, sort_keys=True)
    with lock:
        sys.stdout.write(text + '\n')
        sys.stdout.flush()


def _dispatch_rpc(reader, message, write_lock):
    req_id = message.get('id')
    op = message.get('op')
    try:
        if op == 'get_params':
            payload = reader.get_parameters(message.get('node') or '')
        elif op == 'set_param':
            payload = reader.set_parameter(
                message.get('node') or '',
                message.get('name') or '',
                message.get('value'),
                param_type=message.get('type'),
            )
        elif op == 'watch_topic':
            payload = reader.watch_topic(
                message.get('topic') or '',
                msg_type=message.get('type') or '',
            )
        elif op == 'clear_logs':
            reader.clear_logs()
            payload = {'ok': True}
        elif op == 'ping':
            payload = {'ok': True}
        else:
            payload = {'ok': False, 'error': f'Unknown op: {op}'}
    except Exception as exc:  # noqa: BLE001
        payload = {'ok': False, 'error': str(exc), 'reason': str(exc)}
    _emit_json_line(write_lock, {'kind': 'reply', 'id': req_id, 'payload': payload})


def worker_main(args=None):
    """Run one spinning ROS 2 node speaking bidirectional JSON-lines over stdio."""
    if not rclpy_available():
        sys.stdout.write(json.dumps({
            'kind': 'graph',
            'payload': empty_snapshot('discovery', 'rclpy is not available').to_dict(),
        }) + '\n')
        sys.stdout.flush()
        return 1

    write_lock = threading.Lock()
    stop = threading.Event()
    last_graph = {'text': None}

    def on_graph(snapshot):
        snap_dict = snapshot.to_dict()
        if snap_dict.get('error') in PENDING_ERRORS:
            return
        text = json.dumps(snap_dict, sort_keys=True)
        if text == last_graph['text']:
            return
        last_graph['text'] = text
        _emit_json_line(write_lock, {'kind': 'graph', 'payload': snap_dict})

    def on_topic(sample):
        _emit_json_line(write_lock, {'kind': 'topic_sample', 'payload': sample})

    def on_log(entry):
        _emit_json_line(write_lock, {'kind': 'log', 'payload': entry})

    period = getattr(args, 'period', 1.0) if args else 1.0
    poll = max(0.1, min(2.0, float(period or 1.0) * 0.25))
    reader = RclpyReader(
        poll_period=poll,
        on_graph=on_graph,
        on_topic=on_topic,
        on_log=on_log,
    )
    reader.start()

    def stdin_loop():
        try:
            for line in sys.stdin:
                if stop.is_set():
                    break
                text = line.strip()
                if not text:
                    continue
                try:
                    msg = json.loads(text)
                except json.JSONDecodeError:
                    continue
                threading.Thread(
                    target=_dispatch_rpc,
                    args=(reader, msg, write_lock),
                    daemon=True,
                ).start()
        except (OSError, ValueError):
            pass
        finally:
            stop.set()

    if not sys.stdin.isatty():
        threading.Thread(target=stdin_loop, name='mmr-worker-stdin', daemon=True).start()

    try:
        while not stop.is_set():
            stop.wait(0.2)
    except KeyboardInterrupt:
        return 0
    finally:
        stop.set()
        reader.stop()
    return 0


def stream_main(args=None):
    """Stay up as one ROS node and print the discovery cache when it changes."""
    if not rclpy_available():
        json.dump(empty_snapshot('discovery', 'rclpy is not available').to_dict(), sys.stdout)
        sys.stdout.write('\n')
        return 1
    period = getattr(args, 'period', 1.0) if args else 1.0
    reader = start_reader(period)
    stop = threading.Event()
    if not sys.stdin.isatty():
        threading.Thread(target=_watch_stdin_eof, args=(stop,), name='mmr-stream-eof', daemon=True).start()
    last = None
    try:
        while not stop.is_set():
            snap = reader.snapshot()
            if snap.error not in PENDING_ERRORS:
                text = json.dumps(snap.to_dict(), sort_keys=True)
                if text != last:
                    sys.stdout.write(text + '\n')
                    sys.stdout.flush()
                    last = text
            stop.wait(0.2)
    except KeyboardInterrupt:
        return 0
    finally:
        stop.set()
        reader.stop()
    return 0


def snapshot_main(args=None):
    """Print one graph as JSON and exit. Used by CLI smoke checks."""
    period = getattr(args, 'period', 1.0) if args else 1.0
    reader = start_reader(period)
    deadline = time.monotonic() + (2.5 if rclpy_available() else 6.0)
    try:
        snapshot = reader.snapshot()
        while snapshot.error in PENDING_ERRORS and time.monotonic() < deadline:
            time.sleep(0.05)
            snapshot = reader.snapshot()
    finally:
        reader.stop()
    json.dump(snapshot.to_dict(), sys.stdout)
    sys.stdout.write('\n')
    return 1 if snapshot.error else 0


def _publish_loop(reader, hub, stop):
    while not stop.is_set():
        hub.publish(reader.snapshot())
        stop.wait(0.5)


def serve_main(args):
    host = '127.0.0.1'
    port = _free_port(host, args.port)
    hub = Hub()
    hub.publish(empty_snapshot(
        'rclpy' if rclpy_available() else 'discovery',
        'waiting for discovery'))
    reader = start_reader(
        args.period,
        on_graph=hub.publish,
        on_topic=hub.publish_topic,
        on_log=hub.publish_log,
    )
    stop = threading.Event()
    pump = threading.Thread(
        target=_publish_loop, args=(reader, hub, stop),
        name='mmr-publish', daemon=True)
    pump.start()
    server = serve(hub, host, port, reader=reader)
    url = 'http://{}:{}/'.format(host, port)
    print('MMR ROS graph  ' + url)
    print('Leave this running. The page updates while ROS nodes start and stop.')
    print('Ctrl-C closes it.')
    if not args.no_browser:
        webbrowser.open(url)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('\nstopped')
    finally:
        stop.set()
        hub.close()
        reader.stop()
        server.shutdown()
        server.server_close()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Open a live picture of the ROS 2 graph.')
    parser.add_argument(
        'command',
        nargs='?',
        default='serve',
        choices=('serve', 'snapshot', 'stream', 'worker'),
    )
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument(
        '--period', type=float, default=1.0,
        help='Graph polling cadence in seconds.')
    parser.add_argument(
        '--no-browser', action='store_true',
        help='Do not open a window. Print the address only.')
    args = parser.parse_args(argv)
    if args.command == 'snapshot':
        return snapshot_main(args)
    if args.command == 'stream':
        return stream_main(args)
    if args.command == 'worker':
        return worker_main(args)
    return serve_main(args)


if __name__ == '__main__':
    sys.exit(main())
