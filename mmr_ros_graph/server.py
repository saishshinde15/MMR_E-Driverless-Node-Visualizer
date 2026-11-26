"""Local HTTP + SSE server for graph view, parameters, topic monitor, and /rosout logs."""

from collections import deque
import json
import threading
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs
from urllib.parse import urlparse


STATIC = Path(__file__).resolve().parent / 'static'


class Hub:
    """Latest snapshot + ordered SSE event ring buffer for connected pages."""

    def __init__(self, max_events=400):
        self._condition = threading.Condition()
        self._version = 0
        self._payload = '{}'
        self._seq = 0
        self._events = deque(maxlen=max_events)
        self._closed = False

    def publish(self, snapshot):
        text = json.dumps(snapshot.to_dict(), sort_keys=True)
        with self._condition:
            if text == self._payload:
                return
            self._payload = text
            self._version += 1
            self._seq += 1
            self._events.append((self._seq, 'graph', text))
            self._condition.notify_all()

    def publish_topic(self, state):
        text = json.dumps(state, sort_keys=True)
        with self._condition:
            self._seq += 1
            self._events.append((self._seq, 'topic', text))
            self._condition.notify_all()

    def publish_log(self, entry):
        text = json.dumps(entry, sort_keys=True)
        with self._condition:
            self._seq += 1
            self._events.append((self._seq, 'log', text))
            self._condition.notify_all()

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def current(self):
        with self._condition:
            return self._version, self._payload

    def snapshot_and_seq(self):
        with self._condition:
            return self._seq, self._payload, self._closed

    def wait_after(self, version, timeout):
        with self._condition:
            if self._version == version and not self._closed:
                self._condition.wait(timeout=timeout)
            return self._version, self._payload

    def wait_events(self, last_seq, timeout):
        with self._condition:
            if self._seq == last_seq and not self._closed:
                self._condition.wait(timeout=timeout)
            if self._closed:
                return [], self._seq, True
            events = [item for item in self._events if item[0] > last_seq]
            return events, self._seq, False


def _read_json_body(handler):
    length = int(handler.headers.get('Content-Length') or 0)
    if length <= 0 or length > 1_048_576:
        raise ValueError('Invalid or missing request body')
    raw = handler.rfile.read(length)
    data = json.loads(raw.decode('utf-8'))
    if not isinstance(data, dict):
        raise ValueError('JSON object expected')
    return data


def make_handler(hub, reader=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            parsed = urlparse(self.path)
            path = parsed.path
            query = parse_qs(parsed.query)

            if path in ('/', '/index.html'):
                self._file(STATIC / 'index.html', 'text/html; charset=utf-8')
            elif path == '/style.css':
                self._file(STATIC / 'style.css', 'text/css; charset=utf-8')
            elif path == '/app.js':
                self._file(STATIC / 'app.js', 'text/javascript; charset=utf-8')
            elif path == '/api/graph':
                _version, payload = hub.current()
                self._bytes(payload.encode('utf-8'), 'application/json')
            elif path == '/api/export':
                _version, payload = hub.current()
                pretty = json.dumps(json.loads(payload), indent=2).encode('utf-8')
                self._bytes(
                    pretty,
                    'application/json',
                    extra_headers={'Content-Disposition': 'attachment; filename="mmr-ros-graph.json"'},
                )
            elif path == '/api/stream':
                self._stream()
            elif path == '/api/params':
                node = (query.get('node') or [''])[0].strip()
                if not node:
                    self._json({'error': 'Query parameter "node" is required', 'parameters': []}, status=400)
                    return
                if reader is None:
                    self._json({'node': node, 'parameters': [], 'error': 'Reader unavailable'}, status=503)
                    return
                self._json(reader.get_parameters(node))
            elif path == '/api/topic/state':
                if reader is None:
                    self._json({'topic': '', 'active': False, 'error': 'Reader unavailable'}, status=503)
                    return
                self._json(reader.topic_state())
            elif path == '/api/logs':
                if reader is None:
                    self._json({'logs': []})
                    return
                node = (query.get('node') or [''])[0].strip()
                raw_level = (query.get('min_level') or ['0'])[0]
                try:
                    min_level = int(raw_level)
                except ValueError:
                    min_level = 0
                self._json({'logs': reader.get_logs(node=node, min_level=min_level)})
            else:
                self.send_error(404)

        def do_POST(self):
            parsed = urlparse(self.path)
            path = parsed.path

            if path == '/api/params':
                if reader is None:
                    self._json({'ok': False, 'reason': 'Reader unavailable'}, status=503)
                    return
                try:
                    body = _read_json_body(self)
                except (ValueError, json.JSONDecodeError) as exc:
                    self._json({'ok': False, 'reason': str(exc)}, status=400)
                    return
                node = (body.get('node') or '').strip()
                name = (body.get('name') or '').strip()
                if not node or not name or 'value' not in body:
                    self._json({'ok': False, 'reason': 'Fields "node", "name", and "value" are required'}, status=400)
                    return
                result = reader.set_parameter(node, name, body.get('value'), param_type=body.get('type'))
                self._json(result)
            elif path == '/api/topic/watch':
                if reader is None:
                    self._json({'active': False, 'error': 'Reader unavailable'}, status=503)
                    return
                try:
                    body = _read_json_body(self)
                except (ValueError, json.JSONDecodeError) as exc:
                    self._json({'active': False, 'error': str(exc)}, status=400)
                    return
                topic = (body.get('topic') or '').strip()
                msg_type = (body.get('type') or '').strip()
                self._json(reader.watch_topic(topic, msg_type=msg_type))
            elif path == '/api/logs/clear':
                if reader is not None:
                    reader.clear_logs()
                self._json({'ok': True})
            else:
                self.send_error(404)

        def _file(self, path, content_type):
            if not path.is_file():
                self.send_error(404)
                return
            self._bytes(path.read_bytes(), content_type)

        def _json(self, obj, status=200):
            data = json.dumps(obj).encode('utf-8')
            self._bytes(data, 'application/json', status=status)

        def _bytes(self, data, content_type, status=200, extra_headers=None):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            if extra_headers:
                for key, val in extra_headers.items():
                    self.send_header(key, val)
            self.end_headers()
            self.wfile.write(data)

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                cursor, payload, closed = hub.snapshot_and_seq()
                if closed:
                    return
                self.wfile.write(('data: ' + payload + '\n\n').encode('utf-8'))
                self.wfile.flush()
                while True:
                    events, cursor, closed = hub.wait_events(cursor, timeout=15)
                    if closed:
                        return
                    if not events:
                        self.wfile.write(b': keepalive\n\n')
                        self.wfile.flush()
                        continue
                    for _seq, event_name, event_payload in events:
                        if event_name == 'graph':
                            chunk = 'data: ' + event_payload + '\n\n'
                        else:
                            chunk = f'event: {event_name}\ndata: {event_payload}\n\n'
                        self.wfile.write(chunk.encode('utf-8'))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

    return Handler


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def serve(hub, host, port, reader=None):
    """Listen until the process is stopped. Returns the HTTP server."""
    server = ReusableThreadingHTTPServer((host, port), make_handler(hub, reader=reader))
    thread = threading.Thread(target=server.serve_forever, name='mmr-http', daemon=True)
    thread.start()
    return server
