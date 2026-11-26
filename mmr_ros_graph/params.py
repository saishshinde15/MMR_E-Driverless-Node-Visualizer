"""ROS 2 node parameter reading and writing via rcl_interfaces services."""

import json
import time


PARAM_TYPE_NAMES = {
    0: 'not_set',
    1: 'bool',
    2: 'int',
    3: 'double',
    4: 'string',
    5: 'byte_array',
    6: 'bool_array',
    7: 'int_array',
    8: 'double_array',
    9: 'string_array',
}

TYPE_IDS_BY_NAME = {name: type_id for type_id, name in PARAM_TYPE_NAMES.items()}


def normalize_node_name(node_name):
    """Ensure a ROS 2 node name has a single leading slash."""
    cleaned = (node_name or '').strip()
    if not cleaned:
        raise ValueError('node name is required')
    return '/' + cleaned.lstrip('/')


def decode_parameter_value(param_value):
    """Extract (type_name, python_value) from rcl_interfaces.msg.ParameterValue."""
    type_id = int(getattr(param_value, 'type', 0))
    type_name = PARAM_TYPE_NAMES.get(type_id, 'not_set')
    if type_id == 0:
        return type_name, None
    if type_id == 1:
        return type_name, bool(param_value.bool_value)
    if type_id == 2:
        return type_name, int(param_value.integer_value)
    if type_id == 3:
        return type_name, float(param_value.double_value)
    if type_id == 4:
        return type_name, str(param_value.string_value)
    if type_id == 5:
        raw = getattr(param_value, 'byte_array_value', ()) or ()
        return type_name, [
            int.from_bytes(item, 'little') if isinstance(item, (bytes, bytearray)) else int(item)
            for item in raw
        ]
    if type_id == 6:
        return type_name, [bool(x) for x in (param_value.bool_array_value or ())]
    if type_id == 7:
        return type_name, [int(x) for x in (param_value.integer_array_value or ())]
    if type_id == 8:
        return type_name, [float(x) for x in (param_value.double_array_value or ())]
    if type_id == 9:
        return type_name, [str(x) for x in (param_value.string_array_value or ())]
    return type_name, None


def _coerce_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ('true', '1', 'yes', 'on'):
            return True
        if text in ('false', '0', 'no', 'off'):
            return False
    raise ValueError(f'Cannot interpret {value!r} as bool')


def _coerce_array(value, item_fn):
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return []
        try:
            parsed = json.loads(stripped)
        except json.JSONDecodeError:
            parsed = [part.strip() for part in stripped.split(',') if part.strip()]
    elif isinstance(value, (list, tuple)):
        parsed = list(value)
    else:
        raise ValueError(f'Expected list or JSON array, got {type(value).__name__}')
    return [item_fn(item) for item in parsed]


def infer_type_name(value):
    """Infer a ROS parameter type name from a Python value."""
    if isinstance(value, bool):
        return 'bool'
    if isinstance(value, int):
        return 'int'
    if isinstance(value, float):
        return 'double'
    if isinstance(value, str):
        return 'string'
    if isinstance(value, (list, tuple)):
        if not value:
            return 'string_array'
        first = value[0]
        if isinstance(first, bool):
            return 'bool_array'
        if isinstance(first, int):
            return 'int_array'
        if isinstance(first, float):
            return 'double_array'
        return 'string_array'
    raise ValueError(f'Unsupported parameter value type: {type(value).__name__}')


def build_parameter_value(type_name, raw_value):
    """Build (rcl_interfaces.msg.ParameterValue, resolved_type_name, coerced_py_value)."""
    from rcl_interfaces.msg import ParameterValue

    resolved = (type_name or '').strip().lower()
    if not resolved or resolved == 'auto':
        resolved = infer_type_name(raw_value)
    if resolved not in TYPE_IDS_BY_NAME:
        raise ValueError(f'Unknown parameter type: {type_name}')

    pv = ParameterValue()
    pv.type = TYPE_IDS_BY_NAME[resolved]

    if resolved == 'not_set':
        return pv, resolved, None
    if resolved == 'bool':
        val = _coerce_bool(raw_value)
        pv.bool_value = val
        return pv, resolved, val
    if resolved == 'int':
        if isinstance(raw_value, bool):
            raise ValueError('Boolean cannot be coerced to int')
        val = int(raw_value)
        pv.integer_value = val
        return pv, resolved, val
    if resolved == 'double':
        if isinstance(raw_value, bool):
            raise ValueError('Boolean cannot be coerced to double')
        val = float(raw_value)
        pv.double_value = val
        return pv, resolved, val
    if resolved == 'string':
        val = '' if raw_value is None else str(raw_value)
        pv.string_value = val
        return pv, resolved, val
    if resolved == 'byte_array':
        ints = _coerce_array(raw_value, lambda x: int(x) & 0xFF)
        pv.byte_array_value = [bytes([b]) for b in ints]
        return pv, resolved, ints
    if resolved == 'bool_array':
        vals = _coerce_array(raw_value, _coerce_bool)
        pv.bool_array_value = vals
        return pv, resolved, vals
    if resolved == 'int_array':
        vals = _coerce_array(raw_value, int)
        pv.integer_array_value = vals
        return pv, resolved, vals
    if resolved == 'double_array':
        vals = _coerce_array(raw_value, float)
        pv.double_array_value = vals
        return pv, resolved, vals
    if resolved == 'string_array':
        vals = _coerce_array(raw_value, str)
        pv.string_array_value = vals
        return pv, resolved, vals
    raise ValueError(f'Unsupported parameter type: {resolved}')


def format_descriptor(descriptor):
    """Convert ParameterDescriptor to a JSON-serializable metadata dict."""
    int_ranges = getattr(descriptor, 'integer_range', ()) or ()
    flt_ranges = getattr(descriptor, 'floating_point_range', ()) or ()
    int_range = None
    if int_ranges:
        item = int_ranges[0]
        int_range = {
            'from': int(item.from_value),
            'to': int(item.to_value),
            'step': int(item.step),
        }
    flt_range = None
    if flt_ranges:
        item = flt_ranges[0]
        flt_range = {
            'from': float(item.from_value),
            'to': float(item.to_value),
            'step': float(item.step),
        }
    return {
        'name': str(descriptor.name),
        'type': PARAM_TYPE_NAMES.get(int(getattr(descriptor, 'type', 0)), 'not_set'),
        'description': str(getattr(descriptor, 'description', '') or ''),
        'additional_constraints': str(getattr(descriptor, 'additional_constraints', '') or ''),
        'read_only': bool(getattr(descriptor, 'read_only', False)),
        'dynamic_typing': bool(getattr(descriptor, 'dynamic_typing', False)),
        'integer_range': int_range,
        'floating_point_range': flt_range,
    }


def merge_parameter_details(names, descriptors, values):
    """Combine parallel list/describe/get responses into sorted UI dicts."""
    items = []
    for index, name in enumerate(names):
        desc = format_descriptor(descriptors[index]) if index < len(descriptors) else {
            'name': name,
            'type': 'not_set',
            'description': '',
            'additional_constraints': '',
            'read_only': False,
            'dynamic_typing': False,
            'integer_range': None,
            'floating_point_range': None,
        }
        if index < len(values):
            val_type, py_val = decode_parameter_value(values[index])
        else:
            val_type, py_val = desc['type'], None
        resolved_type = val_type if val_type != 'not_set' else desc['type']
        items.append({
            'name': name,
            'type': resolved_type,
            'value': py_val,
            'read_only': desc['read_only'],
            'dynamic_typing': desc['dynamic_typing'],
            'description': desc['description'],
            'additional_constraints': desc['additional_constraints'],
            'integer_range': desc['integer_range'],
            'floating_point_range': desc['floating_point_range'],
        })
    items.sort(key=lambda entry: entry['name'])
    return items


class AsyncServiceRunner:
    """Non-blocking helper to run rclpy service calls on an existing executor loop."""

    def __init__(self, node):
        self._node = node
        self._pending = []

    def call(self, srv_type, srv_name, request, on_done, timeout=3.0):
        client = self._node.create_client(srv_type, srv_name)
        deadline = time.monotonic() + timeout
        self._pending.append({
            'client': client,
            'srv_name': srv_name,
            'request': request,
            'on_done': on_done,
            'deadline': deadline,
            'future': None,
            'finished': False,
        })
        self.tick()

    def tick(self):
        now = time.monotonic()
        still_pending = []
        for entry in self._pending:
            if entry['finished']:
                continue
            client = entry['client']
            future = entry['future']

            if now > entry['deadline']:
                self._finish(entry, None, TimeoutError(f"Service timeout: {entry['srv_name']}"))
                continue

            if future is None:
                if client.service_is_ready():
                    future = client.call_async(entry['request'])
                    entry['future'] = future
                    future.add_done_callback(lambda fut, item=entry: self._on_future(item, fut))
                still_pending.append(entry)
                continue

            if future.done():
                self._on_future(entry, future)
            else:
                still_pending.append(entry)

        self._pending = still_pending

    def cancel_all(self):
        for entry in self._pending:
            if not entry['finished']:
                self._finish(entry, None, RuntimeError('Cancelled'))
        self._pending = []

    def _on_future(self, entry, future):
        if entry['finished']:
            return
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish(entry, None, exc)
            return
        self._finish(entry, result, None)

    def _finish(self, entry, result, error):
        entry['finished'] = True
        try:
            self._node.destroy_client(entry['client'])
        except Exception:  # noqa: BLE001
            pass
        entry['on_done'](result, error)


def start_get_parameters(runner, node_name, on_done, timeout=4.0):
    """Fetch all parameter names, descriptors, and values for target node."""
    from rcl_interfaces.srv import DescribeParameters
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.srv import ListParameters

    try:
        target = normalize_node_name(node_name)
    except ValueError as exc:
        on_done({'node': node_name, 'parameters': [], 'error': str(exc)})
        return

    def on_list(response, error):
        if error is not None or response is None:
            on_done({'node': target, 'parameters': [], 'error': str(error or 'list_parameters failed')})
            return
        names = list(getattr(response.result, 'names', ()) or ())
        if not names:
            on_done({'node': target, 'parameters': [], 'error': ''})
            return

        state = {'desc': None, 'vals': None, 'done': False, 'errors': []}

        def maybe_complete():
            if state['done']:
                return
            if state['errors'] and (state['desc'] is None or state['vals'] is None):
                state['done'] = True
                on_done({'node': target, 'parameters': [], 'error': state['errors'][0]})
                return
            if state['desc'] is not None and state['vals'] is not None:
                state['done'] = True
                params = merge_parameter_details(names, state['desc'], state['vals'])
                on_done({'node': target, 'parameters': params, 'error': ''})

        def on_describe(desc_resp, desc_err):
            if desc_err is not None or desc_resp is None:
                state['errors'].append(str(desc_err or 'describe_parameters failed'))
                state['desc'] = []
            else:
                state['desc'] = list(desc_resp.descriptors or ())
            maybe_complete()

        def on_get(get_resp, get_err):
            if get_err is not None or get_resp is None:
                state['errors'].append(str(get_err or 'get_parameters failed'))
                state['vals'] = []
            else:
                state['vals'] = list(get_resp.values or ())
            maybe_complete()

        runner.call(
            DescribeParameters,
            f'{target}/describe_parameters',
            DescribeParameters.Request(names=names),
            on_describe,
            timeout=timeout,
        )
        runner.call(
            GetParameters,
            f'{target}/get_parameters',
            GetParameters.Request(names=names),
            on_get,
            timeout=timeout,
        )

    runner.call(
        ListParameters,
        f'{target}/list_parameters',
        ListParameters.Request(depth=0),
        on_list,
        timeout=timeout,
    )


def start_set_parameter(runner, node_name, name, value, on_done, param_type=None, timeout=4.0):
    """Update a single parameter on target node."""
    from rcl_interfaces.msg import Parameter
    from rcl_interfaces.srv import GetParameterTypes
    from rcl_interfaces.srv import SetParameters

    try:
        target = normalize_node_name(node_name)
    except ValueError as exc:
        on_done({'ok': False, 'reason': str(exc), 'node': node_name, 'name': name})
        return

    if not name:
        on_done({'ok': False, 'reason': 'parameter name is required', 'node': target, 'name': name})
        return

    def send_set(resolved_type_name):
        try:
            pv, final_type, coerced = build_parameter_value(resolved_type_name, value)
        except ValueError as exc:
            on_done({'ok': False, 'reason': str(exc), 'node': target, 'name': name})
            return

        req = SetParameters.Request(parameters=[Parameter(name=name, value=pv)])

        def on_set(resp, err):
            if err is not None or resp is None or not resp.results:
                on_done({
                    'ok': False,
                    'reason': str(err or 'set_parameters failed'),
                    'node': target,
                    'name': name,
                })
                return
            result = resp.results[0]
            on_done({
                'ok': bool(result.successful),
                'reason': str(result.reason or ''),
                'node': target,
                'name': name,
                'type': final_type,
                'value': coerced,
            })

        runner.call(
            SetParameters,
            f'{target}/set_parameters',
            req,
            on_set,
            timeout=timeout,
        )

    if param_type and param_type not in ('auto', 'not_set'):
        send_set(param_type)
        return

    def on_types(resp, err):
        if err is None and resp is not None and resp.types:
            type_id = int(resp.types[0])
            detected = PARAM_TYPE_NAMES.get(type_id, '')
            if detected and detected != 'not_set':
                send_set(detected)
                return
        send_set(param_type)

    runner.call(
        GetParameterTypes,
        f'{target}/get_parameter_types',
        GetParameterTypes.Request(names=[name]),
        on_types,
        timeout=timeout,
    )


def _run_oneshot(callback_starter, timeout=4.0):
    import threading
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node('mmr_ros_graph_param_cli', context=ctx, enable_rosout=False)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    runner = AsyncServiceRunner(node)
    done = threading.Event()
    box = {}

    def finish(payload):
        box['payload'] = payload
        done.set()

    try:
        callback_starter(runner, finish)
        deadline = time.monotonic() + timeout + 0.5
        while not done.is_set() and time.monotonic() < deadline:
            runner.tick()
            executor.spin_once(timeout_sec=0.05)
        if not done.is_set():
            return {'error': 'Timed out waiting for parameter service'}
        return box['payload']
    finally:
        runner.cancel_all()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok(context=ctx):
            rclpy.shutdown(context=ctx)


def get_parameters(node_name, timeout=4.0):
    """Return the parameters of one node synchronously."""
    return _run_oneshot(
        lambda runner, finish: start_get_parameters(runner, node_name, finish, timeout=timeout),
        timeout=timeout,
    )


def set_parameter(node_name, name, value, param_type=None, timeout=4.0):
    """Change one parameter on a running node synchronously."""
    return _run_oneshot(
        lambda runner, finish: start_set_parameter(
            runner, node_name, name, value, finish, param_type=param_type, timeout=timeout),
        timeout=timeout,
    )
