"""Data model and serialization for the ROS 2 discovery graph."""

from dataclasses import dataclass

SYSTEM_TOPICS = ('/rosout', '/parameter_events')
OWN_NODE = '/mmr_ros_graph'
PARAM_SERVICE_SUFFIXES = (
    '/describe_parameters',
    '/get_parameter_types',
    '/get_parameters',
    '/list_parameters',
    '/set_parameters',
    '/set_parameters_atomically',
)
ACTION_SERVICE_SUFFIXES = (
    '/_action/send_goal',
    '/_action/cancel_goal',
    '/_action/get_result',
)
ACTION_TOPIC_SUFFIXES = (
    '/_action/feedback',
    '/_action/status',
)


@dataclass(frozen=True)
class Endpoint:
    """One publisher or subscriber announced for a topic."""

    node: str
    reliability: str = ''
    durability: str = ''
    history: str = ''
    depth: int = 0
    liveliness: str = ''
    deadline_ns: int = 0
    lifespan_ns: int = 0

    def qos_dict(self):
        return {
            'reliability': self.reliability,
            'durability': self.durability,
            'history': self.history,
            'depth': self.depth,
            'liveliness': self.liveliness,
            'deadline_ns': self.deadline_ns,
            'lifespan_ns': self.lifespan_ns,
        }

    def qos_key(self):
        return (
            self.node,
            self.reliability,
            self.durability,
            self.liveliness,
            self.deadline_ns,
            self.lifespan_ns,
            self.history,
            self.depth,
        )

    def qos_summary(self):
        parts = [self.reliability or '?', self.durability or '?']
        if self.liveliness and self.liveliness not in ('SYSTEM_DEFAULT', 'AUTOMATIC'):
            parts.append(self.liveliness)
        if self.deadline_ns:
            parts.append(f'deadline={self.deadline_ns // 1_000_000}ms')
        return ' / '.join(parts)


@dataclass(frozen=True)
class ServiceEntry:
    """A ROS 2 service server or client attached to a node."""

    name: str
    msg_type: str
    system: bool = False

    def to_dict(self):
        return {
            'name': self.name,
            'type': self.msg_type,
            'system': self.system,
        }


@dataclass(frozen=True)
class ActionEntry:
    """A ROS 2 action server or client reconstructed from /_action endpoints."""

    name: str
    action_type: str
    role: str = 'server'

    def to_dict(self):
        return {
            'name': self.name,
            'type': self.action_type,
            'role': self.role,
        }


@dataclass(frozen=True)
class NodeInfo:
    """A discovered ROS 2 node and its non-topic introspection metadata."""

    id: str
    namespace: str = '/'
    name: str = ''
    count: int = 1
    services: tuple = ()
    clients: tuple = ()
    actions: tuple = ()

    def to_dict(self):
        ns, short = split_node_name(self.id)
        return {
            'id': self.id,
            'namespace': self.namespace or ns,
            'name': self.name or short,
            'count': self.count,
            'services': [item.to_dict() for item in self.services],
            'clients': [item.to_dict() for item in self.clients],
            'actions': [item.to_dict() for item in self.actions],
        }


@dataclass(frozen=True)
class TopicInfo:
    """A discovered topic and the endpoints attached to it."""

    name: str
    msg_type: str
    publishers: tuple
    subscribers: tuple
    system: bool = False
    action_internal: bool = False

    def is_system(self):
        return self.system or (self.name in SYSTEM_TOPICS)

    def is_action_internal(self):
        return self.action_internal or is_action_topic(self.name)


@dataclass(frozen=True)
class Link:
    """One discovered publisher connected to one discovered subscriber."""

    publisher: str
    subscriber: str
    topic: str
    msg_type: str
    compatibility: str = 'ok'
    reason: str = ''
    pub_qos: dict = None
    sub_qos: dict = None
    id: str = ''

    def link_id(self):
        if self.id:
            return self.id
        pub_tag = _qos_tag(self.pub_qos)
        sub_tag = _qos_tag(self.sub_qos)
        return f'{self.publisher}->{self.subscriber}|{self.topic}|{pub_tag}|{sub_tag}'


@dataclass(frozen=True)
class Diagnostic:
    """Health or compatibility issue detected on the graph."""

    level: str
    kind: str
    target: str
    message: str

    def to_dict(self):
        return {
            'level': self.level,
            'kind': self.kind,
            'target': self.target,
            'message': self.message,
        }


@dataclass(frozen=True)
class GraphSnapshot:
    """Nodes, topics, links, and diagnostics from one discovery reading."""

    nodes: tuple
    topics: tuple
    source: str
    error: str = ''
    links: tuple = ()
    diagnostics: tuple = ()
    seq: int = 0

    def to_dict(self):
        return {
            'nodes': [_node_to_dict(item) for item in self.nodes],
            'topics': [
                {
                    'name': topic.name,
                    'type': topic.msg_type,
                    'publishers': [_endpoint_dict(item) for item in topic.publishers],
                    'subscribers': [_endpoint_dict(item) for item in topic.subscribers],
                    'system': topic.is_system(),
                    'action_internal': topic.is_action_internal(),
                }
                for topic in self.topics
            ],
            'links': [
                {
                    'id': link.link_id(),
                    'publisher': link.publisher,
                    'subscriber': link.subscriber,
                    'topic': link.topic,
                    'type': link.msg_type,
                    'compatibility': link.compatibility,
                    'reason': link.reason,
                    'pub_qos': link.pub_qos or {},
                    'sub_qos': link.sub_qos or {},
                }
                for link in self.links
            ],
            'diagnostics': [diag.to_dict() for diag in self.diagnostics],
            'source': self.source,
            'error': self.error,
            'seq': self.seq,
        }


def _qos_tag(qos):
    if not qos:
        return ''
    return (
        f"{qos.get('reliability', '')}/{qos.get('durability', '')}"
        f"/{qos.get('liveliness', '')}/{qos.get('deadline_ns', 0)}"
    )


def _node_to_dict(item):
    if isinstance(item, str):
        return NodeInfo(item).to_dict()
    return item.to_dict()


def _endpoint_dict(item):
    if isinstance(item, str):
        item = Endpoint(item)
    return {'node': item.node, 'qos': item.qos_dict()}


def is_user_node(name):
    """Ignore viewer nodes, unknown DDS placeholders, and ros2cli daemons."""
    if not name:
        return False
    if '_NODE_NAME_UNKNOWN_' in name or '_NODE_NAMESPACE_UNKNOWN_' in name:
        return False
    if name == OWN_NODE or name.startswith(OWN_NODE + '_'):
        return False
    return not name.startswith('/_ros2cli_')


def is_system_service(name):
    """Return True for boilerplate node parameter services."""
    return name.endswith(PARAM_SERVICE_SUFFIXES)


def is_action_topic(name):
    """Return True for internal ROS 2 action feedback/status topics."""
    return name.endswith(ACTION_TOPIC_SUFFIXES)


def _strip_action_suffix(msg_type, suffix):
    if isinstance(msg_type, (list, tuple)):
        msg_type = msg_type[0] if msg_type else 'unknown'
    if msg_type.endswith(suffix):
        return msg_type[:-len(suffix)]
    return msg_type


def _norm_type(msg_type):
    if isinstance(msg_type, (list, tuple)):
        return msg_type[0] if msg_type else 'unknown'
    return msg_type or 'unknown'


def extract_actions(service_pairs, client_pairs):
    """Separate ROS 2 action endpoints from plain services/clients.

    Returns (actions_tuple, plain_services_tuple, plain_clients_tuple).
    """
    action_roles = {}
    plain_services = []
    plain_clients = []

    for pairs, role_slot, target_list in (
        (service_pairs, 1, plain_services),
        (client_pairs, 2, plain_clients),
    ):
        for name, raw_type in pairs:
            msg_type = _norm_type(raw_type)
            if name.endswith(ACTION_SERVICE_SUFFIXES):
                if name.endswith('/_action/send_goal'):
                    action_name = name[:-len('/_action/send_goal')]
                    action_type = _strip_action_suffix(msg_type, '_SendGoal')
                    entry = action_roles.setdefault(action_name, [action_type, False, False])
                    entry[0] = entry[0] or action_type
                    entry[role_slot] = True
                continue
            target_list.append(ServiceEntry(name, msg_type, is_system_service(name)))

    actions = []
    for action_name in sorted(action_roles):
        action_type, is_srv, is_cli = action_roles[action_name]
        role = 'both' if (is_srv and is_cli) else ('server' if is_srv else 'client')
        actions.append(ActionEntry(action_name, action_type, role))

    plain_services.sort(key=lambda item: (item.system, item.name))
    plain_clients.sort(key=lambda item: (item.system, item.name))
    return tuple(actions), tuple(plain_services), tuple(plain_clients)


def unique_endpoints(items):
    """One announcement per node and QoS. DDS often reports the same end twice."""
    seen = set()
    unique = []
    for item in items:
        key = item.qos_key()
        if key in seen or not is_user_node(item.node):
            continue
        seen.add(key)
        unique.append(item)
    return tuple(unique)


def split_node_name(full_name):
    """Split a full ROS node name into (namespace, short_name)."""
    if not full_name:
        return '/', ''
    cleaned = '/' + full_name.lstrip('/')
    idx = cleaned.rfind('/')
    if idx <= 0:
        return '/', cleaned.lstrip('/')
    return cleaned[:idx], cleaned[idx + 1:]


def join_node_name(namespace, name):
    """Build a full node name from a namespace and a short name."""
    if not name:
        return ''
    if name.startswith('/'):
        return name
    short = name.lstrip('/')
    if namespace in ('', '/'):
        return '/' + short
    return namespace.rstrip('/') + '/' + short


def qos_mismatch(publisher, subscriber):
    """Reasons these two ends will not exchange messages.

    A subscriber only matches a publisher that offers at least the
    reliability and durability the subscriber asked for.
    """
    reasons = []
    if not publisher.reliability or not subscriber.reliability:
        return reasons
    if subscriber.reliability == 'RELIABLE' and publisher.reliability != 'RELIABLE':
        reasons.append(
            'subscriber is RELIABLE, publisher is ' + publisher.reliability)
    if subscriber.durability == 'TRANSIENT_LOCAL' and publisher.durability != 'TRANSIENT_LOCAL':
        reasons.append(
            'subscriber is TRANSIENT_LOCAL, publisher is ' + (publisher.durability or 'VOLATILE'))
    return reasons


def empty_snapshot(source, error=''):
    return GraphSnapshot((), (), source, error)


def _service_from_json(entry):
    return ServiceEntry(
        entry.get('name') or '',
        entry.get('type') or 'unknown',
        bool(entry.get('system', False)),
    )


def _action_from_json(entry):
    return ActionEntry(
        entry.get('name') or '',
        entry.get('type') or 'unknown',
        entry.get('role') or 'server',
    )


def _node_from_json(entry):
    if isinstance(entry, str):
        return NodeInfo(entry)
    node_id = entry.get('id') or ''
    ns, short = split_node_name(node_id)
    return NodeInfo(
        node_id,
        entry.get('namespace') or ns,
        entry.get('name') or short,
        int(entry.get('count') or 1),
        tuple(_service_from_json(item) for item in entry.get('services') or []),
        tuple(_service_from_json(item) for item in entry.get('clients') or []),
        tuple(_action_from_json(item) for item in entry.get('actions') or []),
    )


def snapshot_from_payload(payload):
    """Rebuild a snapshot from the JSON a discovery node prints."""
    topics = []
    for item in payload.get('topics') or []:
        name = item['name']
        topics.append(TopicInfo(
            name,
            item.get('type') or 'unknown',
            tuple(_endpoint_from_json(entry) for entry in item.get('publishers') or []),
            tuple(_endpoint_from_json(entry) for entry in item.get('subscribers') or []),
            bool(item.get('system', name in SYSTEM_TOPICS)),
            bool(item.get('action_internal', is_action_topic(name))),
        ))
    nodes = tuple(_node_from_json(entry) for entry in payload.get('nodes') or [])
    links = tuple(
        Link(
            item.get('publisher') or '',
            item.get('subscriber') or '',
            item.get('topic') or '',
            item.get('type') or '',
            item.get('compatibility') or 'ok',
            item.get('reason') or '',
            item.get('pub_qos') or {},
            item.get('sub_qos') or {},
            item.get('id') or '',
        )
        for item in payload.get('links') or []
    )
    diagnostics = tuple(
        Diagnostic(
            item.get('level') or 'warning',
            item.get('kind') or '',
            item.get('target') or '',
            item.get('message') or '',
        )
        for item in payload.get('diagnostics') or []
    )
    return GraphSnapshot(
        nodes,
        tuple(topics),
        payload.get('source') or 'discovery',
        payload.get('error') or '',
        links,
        diagnostics,
        int(payload.get('seq') or 0),
    )


def _endpoint_from_json(entry):
    if isinstance(entry, str):
        return Endpoint(entry)
    qos = entry.get('qos') or {}
    return Endpoint(
        entry.get('node') or '',
        qos.get('reliability') or '',
        qos.get('durability') or '',
        qos.get('history') or '',
        int(qos.get('depth') or 0),
        qos.get('liveliness') or '',
        int(qos.get('deadline_ns') or 0),
        int(qos.get('lifespan_ns') or 0),
    )
