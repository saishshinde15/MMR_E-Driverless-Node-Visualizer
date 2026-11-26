"""Keep one discovery/worker node alive and exchange graph + RPC messages."""

from collections import deque
import json
import shutil
import subprocess
import threading

from mmr_ros_graph.model import empty_snapshot
from mmr_ros_graph.model import snapshot_from_payload
from mmr_ros_graph.rclpy_backend import RclpyReader
from mmr_ros_graph.rclpy_backend import rclpy_available


class DiscoveryStreamReader:
    """Persistent bridge to `ros2 run mmr_ros_graph live_graph worker`.

    Runs one warm ROS node inside the ROS environment (such as the Linux VM
    on macOS) and exchanges JSON-lines over stdin/stdout so graph discovery,
    parameter reads/writes, topic monitoring, and /rosout logs all reuse the
    same spinning node without spawning per-request CLI processes.
    """

    def __init__(self, period=1.0, on_graph=None, on_topic=None, on_log=None, command=None):
        self._period = float(period or 1.0)
        self._on_graph = on_graph
        self._on_topic = on_topic
        self._on_log = on_log
        self._custom_command = command

        self._snapshot = empty_snapshot('discovery', 'waiting for discovery')
        self._logs = deque(maxlen=300)
        self._topic_state = {
            'topic': '',
            'type': '',
            'active': False,
            'hz': 0.0,
            'kbps': 0.0,
            'count': 0,
            'sample': None,
            'error': '',
        }
        self._stderr_tail = deque(maxlen=20)
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._rpc_lock = threading.Lock()
        self._pending_rpc = {}
        self._next_rpc_id = 0
        self._stop = threading.Event()
        self._proc = None
        self._stdout_thread = None
        self._stderr_thread = None

    def start(self):
        cmd = self._build_command()
        if cmd is None:
            self._store(empty_snapshot('discovery', 'ros2 was not found on PATH'))
            return
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            self._store(empty_snapshot('discovery', f'failed to start discovery worker: {exc}'))
            return
        self._stdout_thread = threading.Thread(
            target=self._run_stdout, name='mmr-discovery-out', daemon=True)
        self._stderr_thread = threading.Thread(
            target=self._run_stderr, name='mmr-discovery-err', daemon=True)
        self._stdout_thread.start()
        self._stderr_thread.start()

    def _build_command(self):
        if self._custom_command is not None:
            return list(self._custom_command)
        program = shutil.which('ros2')
        if not program:
            return None
        return [
            program,
            'run',
            'mmr_ros_graph',
            'live_graph',
            'worker',
            '--period',
            str(self._period),
        ]

    def stop(self):
        self._stop.set()
        self._fail_pending_rpc('discovery worker stopped')
        proc = self._proc
        if proc is not None:
            # Closing stdin triggers EOF inside the container worker so it exits cleanly.
            if proc.stdin is not None:
                try:
                    proc.stdin.close()
                except OSError:
                    pass
            if proc.poll() is None:
                try:
                    proc.wait(timeout=1.5)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(timeout=1.5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
        if self._stdout_thread is not None:
            self._stdout_thread.join(timeout=2)
        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=1)
        if proc is not None:
            for stream in (proc.stdout, proc.stderr):
                if stream is not None:
                    try:
                        stream.close()
                    except OSError:
                        pass

    def snapshot(self):
        with self._lock:
            return self._snapshot

    def get_logs(self, node='', min_level=0):
        with self._lock:
            items = list(self._logs)
        if node:
            norm = '/' + node.lstrip('/')
            items = [item for item in items if item.get('name') == norm or item.get('name') == node.lstrip('/')]
        if min_level:
            items = [item for item in items if int(item.get('level') or 0) >= int(min_level)]
        return items

    def clear_logs(self):
        with self._lock:
            self._logs.clear()
        if self._proc is not None and self._proc.poll() is None:
            self._rpc('clear_logs', {}, timeout=2.0, fallback={'ok': True})

    def topic_state(self):
        with self._lock:
            return dict(self._topic_state)

    def get_parameters(self, node_name, timeout=5.0):
        return self._rpc(
            'get_params',
            {'node': node_name},
            timeout=timeout,
            fallback={'node': node_name, 'parameters': [], 'error': 'Discovery worker unavailable'},
        )

    def set_parameter(self, node_name, name, value, param_type=None, timeout=5.0):
        return self._rpc(
            'set_param',
            {'node': node_name, 'name': name, 'value': value, 'type': param_type},
            timeout=timeout,
            fallback={'ok': False, 'reason': 'Discovery worker unavailable', 'node': node_name, 'name': name},
        )

    def watch_topic(self, topic_name, msg_type='', timeout=4.0):
        return self._rpc(
            'watch_topic',
            {'topic': topic_name or '', 'type': msg_type or ''},
            timeout=timeout,
            fallback={'topic': topic_name or '', 'active': False, 'error': 'Discovery worker unavailable'},
        )

    def _rpc(self, op, args, timeout, fallback):
        proc = self._proc
        if self._stop.is_set() or proc is None or proc.poll() is not None or proc.stdin is None:
            return fallback

        event = threading.Event()
        box = {}
        with self._rpc_lock:
            self._next_rpc_id += 1
            req_id = self._next_rpc_id
            self._pending_rpc[req_id] = (event, box)

        line = json.dumps({'id': req_id, 'op': op, **args}) + '\n'
        try:
            with self._write_lock:
                proc.stdin.write(line)
                proc.stdin.flush()
        except OSError as exc:
            with self._rpc_lock:
                self._pending_rpc.pop(req_id, None)
            err_copy = dict(fallback)
            if 'error' in err_copy:
                err_copy['error'] = str(exc)
            elif 'reason' in err_copy:
                err_copy['reason'] = str(exc)
            return err_copy

        if not event.wait(timeout=timeout):
            with self._rpc_lock:
                self._pending_rpc.pop(req_id, None)
            err_copy = dict(fallback)
            if 'error' in err_copy:
                err_copy['error'] = f'Timed out waiting for {op}'
            elif 'reason' in err_copy:
                err_copy['reason'] = f'Timed out waiting for {op}'
            return err_copy

        return box.get('payload', fallback)

    def _fail_pending_rpc(self, reason):
        with self._rpc_lock:
            pending = list(self._pending_rpc.items())
            self._pending_rpc.clear()
        for _req_id, (event, box) in pending:
            box['payload'] = {'ok': False, 'error': reason, 'reason': reason, 'parameters': []}
            event.set()

    def _store(self, snapshot):
        with self._lock:
            self._snapshot = snapshot
        if self._on_graph is not None:
            self._on_graph(snapshot)

    def _run_stderr(self):
        stream = self._proc.stderr if self._proc is not None else None
        if stream is None:
            return
        for line in stream:
            text = line.strip()
            if text:
                with self._lock:
                    self._stderr_tail.append(text)

    def _run_stdout(self):
        stream = self._proc.stdout if self._proc is not None else None
        if stream is None:
            return
        for line in stream:
            if self._stop.is_set():
                break
            text = line.strip()
            if not text.startswith('{'):
                continue
            try:
                message = json.loads(text)
            except json.JSONDecodeError:
                continue
            self._handle_message(message)

        if not self._stop.is_set():
            if self._stderr_thread is not None:
                self._stderr_thread.join(timeout=0.3)
            with self._lock:
                detail = ' | '.join(self._stderr_tail)
            reason = f'discovery node stopped: {detail}' if detail else 'discovery node stopped'
            self._fail_pending_rpc(reason)
            self._store(empty_snapshot('discovery', reason))

    def _handle_message(self, message):
        kind = message.get('kind')
        if not kind:
            # Backward-compatible plain snapshot JSON line.
            self._store(snapshot_from_payload(message))
            return

        payload = message.get('payload')
        if kind == 'graph' and isinstance(payload, dict):
            self._store(snapshot_from_payload(payload))
        elif kind == 'topic_sample' and isinstance(payload, dict):
            with self._lock:
                self._topic_state = payload
            if self._on_topic is not None:
                self._on_topic(payload)
        elif kind == 'log' and isinstance(payload, dict):
            with self._lock:
                self._logs.append(payload)
            if self._on_log is not None:
                self._on_log(payload)
        elif kind == 'reply':
            req_id = message.get('id')
            with self._rpc_lock:
                waiter = self._pending_rpc.pop(req_id, None)
            if waiter is not None:
                event, box = waiter
                box['payload'] = payload
                event.set()


def start_reader(period=1.0, on_graph=None, on_topic=None, on_log=None):
    """Start the long-lived reader matching the host environment."""
    poll = max(0.1, min(2.0, float(period or 1.0) * 0.25))
    if rclpy_available():
        reader = RclpyReader(
            poll_period=poll,
            on_graph=on_graph,
            on_topic=on_topic,
            on_log=on_log,
        )
        reader.start()
        return reader
    reader = DiscoveryStreamReader(
        period=period,
        on_graph=on_graph,
        on_topic=on_topic,
        on_log=on_log,
    )
    reader.start()
    return reader
