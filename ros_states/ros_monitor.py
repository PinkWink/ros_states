"""ROS2 State Monitor - uses rclpy graph API + subprocess for Hz."""

import os
import re
import subprocess
import threading
import time
from collections import defaultdict

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.srv import ListParameters, GetParameters
from rcl_interfaces.msg import ParameterType
from tf2_msgs.msg import TFMessage

# Node names to filter out from counts
_INTERNAL_NODES = {'ros_web_monitor'}
_INTERNAL_PREFIX = '_ros2cli_'


def _is_internal(node_name):
    return node_name in _INTERNAL_NODES or node_name.startswith(_INTERNAL_PREFIX)


class RosMonitor:
    def __init__(self):
        self._node = None
        self._spin_thread = None
        self._active = False
        self._domain_id = 0

        # TF data
        self._tf_frames = {}
        self._tf_lock = threading.Lock()

        # Hz cache
        self._hz_cache = {}
        self._hz_lock = threading.Lock()
        self._hz_thread = None

    @property
    def active(self):
        return self._active

    @property
    def domain_id(self):
        return self._domain_id

    def activate(self, domain_id):
        if self._active:
            self.deactivate()

        self._domain_id = int(domain_id)
        os.environ['ROS_DOMAIN_ID'] = str(self._domain_id)

        try:
            rclpy.init()
        except RuntimeError:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            rclpy.init()

        self._node = rclpy.create_node('ros_web_monitor')

        qos_tf = QoSProfile(depth=100)
        qos_tf_static = QoSProfile(
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._node.create_subscription(TFMessage, '/tf', self._tf_cb, qos_tf)
        self._node.create_subscription(TFMessage, '/tf_static', self._tf_static_cb, qos_tf_static)

        self._active = True
        self._tf_frames = {}
        self._hz_cache = {}

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self._hz_thread = threading.Thread(target=self._hz_loop, daemon=True)
        self._hz_thread.start()

    def deactivate(self):
        self._active = False

        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

        try:
            rclpy.shutdown()
        except Exception:
            pass

        with self._tf_lock:
            self._tf_frames.clear()
        with self._hz_lock:
            self._hz_cache.clear()

    def _spin(self):
        while self._active and self._node:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception:
                break

    def _tf_cb(self, msg):
        with self._tf_lock:
            for t in msg.transforms:
                self._tf_frames[t.child_frame_id] = t.header.frame_id

    def _tf_static_cb(self, msg):
        with self._tf_lock:
            for t in msg.transforms:
                self._tf_frames[t.child_frame_id] = t.header.frame_id

    # --- Hz measurement (sequential subprocess, one topic at a time) ---

    def _hz_loop(self):
        """Continuously measure Hz for active topics, one at a time."""
        while self._active:
            topics = self._get_publishable_topics()
            for topic_name in topics:
                if not self._active:
                    break
                self._measure_hz_single(topic_name)
            # If no topics or done with all, wait before next round
            time.sleep(2.0)

    def _get_publishable_topics(self):
        """Get list of topics that have external publishers."""
        if not self._node:
            return []
        try:
            names_and_types = self._node.get_topic_names_and_types()
        except Exception:
            return []

        result = []
        for name, _ in names_and_types:
            try:
                pubs = self._node.get_publishers_info_by_topic(name)
                if any(not _is_internal(p.node_name) for p in pubs):
                    result.append(name)
            except Exception:
                pass
        return result

    def _measure_hz_single(self, topic_name):
        """Measure Hz for a single topic using ros2 topic hz subprocess."""
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(self._domain_id)

        try:
            proc = subprocess.Popen(
                ['ros2', 'topic', 'hz', topic_name, '--window', '5'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, env=env,
            )
            try:
                output, _ = proc.communicate(timeout=4)
            except subprocess.TimeoutExpired:
                proc.kill()
                output, _ = proc.communicate()

            if output:
                matches = re.findall(r'average rate:\s+([\d.]+)', output)
                if matches:
                    with self._hz_lock:
                        self._hz_cache[topic_name] = float(matches[-1])
        except Exception:
            pass

    # --- Node map builder ---

    def _build_node_maps(self):
        """Build mappings: service_name -> server nodes, service_name -> client nodes."""
        server_map = defaultdict(set)
        client_map = defaultdict(set)

        if not self._node:
            return server_map, client_map

        try:
            nodes = self._node.get_node_names_and_namespaces()
        except Exception:
            return server_map, client_map

        for node_name, namespace in nodes:
            if _is_internal(node_name):
                continue
            full_name = namespace.rstrip('/') + '/' + node_name

            try:
                svcs = self._node.get_service_names_and_types_by_node(node_name, namespace)
                for svc_name, _ in svcs:
                    server_map[svc_name].add(full_name)
            except Exception:
                pass

            try:
                clients = self._node.get_client_names_and_types_by_node(node_name, namespace)
                for svc_name, _ in clients:
                    client_map[svc_name].add(full_name)
            except Exception:
                pass

        return server_map, client_map

    # --- API methods ---

    def get_topics(self):
        if not self._active or not self._node:
            return []

        try:
            names_and_types = self._node.get_topic_names_and_types()
        except Exception:
            return []

        topics = []
        for name, types in names_and_types:
            # Skip action-internal topics
            if '/_action/' in name:
                continue
            type_str = types[0] if types else 'unknown'

            try:
                pubs = self._node.get_publishers_info_by_topic(name)
                subs = self._node.get_subscriptions_info_by_topic(name)
            except Exception:
                pubs, subs = [], []

            ext_pubs = [p for p in pubs if not _is_internal(p.node_name)]
            ext_subs = [s for s in subs if not _is_internal(s.node_name)]

            pub_nodes = sorted(set(
                p.node_namespace.rstrip('/') + '/' + p.node_name
                for p in ext_pubs
            ))
            sub_nodes = sorted(set(
                s.node_namespace.rstrip('/') + '/' + s.node_name
                for s in ext_subs
            ))

            with self._hz_lock:
                hz = self._hz_cache.get(name)

            topics.append({
                'name': name,
                'type': type_str,
                'publishers': len(ext_pubs),
                'subscribers': len(ext_subs),
                'hz': round(hz, 1) if hz is not None else None,
                'pub_nodes': pub_nodes,
                'sub_nodes': sub_nodes,
            })

        return topics

    def get_services(self):
        if not self._active or not self._node:
            return []

        try:
            names_and_types = self._node.get_service_names_and_types()
        except Exception:
            return []

        server_map, client_map = self._build_node_maps()

        services = []
        for name, types in names_and_types:
            # Filter out internal nodes and action-internal services
            if '/ros_web_monitor/' in name or '/_ros2cli_' in name:
                continue
            if '/_action/' in name:
                continue
            type_str = types[0] if types else 'unknown'
            services.append({
                'name': name,
                'type': type_str,
                'server_nodes': sorted(server_map.get(name, [])),
                'client_nodes': sorted(client_map.get(name, [])),
            })

        return services

    def get_actions(self):
        """Detect actions by finding _action/send_goal services."""
        if not self._active or not self._node:
            return []

        try:
            names_and_types = self._node.get_service_names_and_types()
        except Exception:
            return []

        server_map, client_map = self._build_node_maps()

        actions = []
        for name, types in names_and_types:
            if name.endswith('/_action/send_goal'):
                action_name = name.rsplit('/_action/send_goal', 1)[0]
                type_str = types[0] if types else 'unknown'
                type_str = re.sub(r'_SendGoal$', '', type_str)
                actions.append({
                    'name': action_name,
                    'type': type_str,
                    'server_nodes': sorted(server_map.get(name, [])),
                    'client_nodes': sorted(client_map.get(name, [])),
                })

        return actions

    def get_tf_tree(self):
        with self._tf_lock:
            frames = dict(self._tf_frames)

        if not frames:
            return {'frames': [], 'tree': []}

        tree = defaultdict(list)
        all_children = set()
        all_parents = set()

        for child, parent in frames.items():
            tree[parent].append(child)
            all_children.add(child)
            all_parents.add(parent)

        roots = all_parents - all_children

        def build_subtree(node):
            children = sorted(tree.get(node, []))
            return {'name': node, 'children': [build_subtree(c) for c in children]}

        return {
            'frames': [{'child': c, 'parent': p} for c, p in sorted(frames.items())],
            'tree': [build_subtree(r) for r in sorted(roots)],
        }

    # --- Parameter API ---

    def get_node_list(self):
        """Get list of all external nodes."""
        if not self._active or not self._node:
            return []
        try:
            nodes = self._node.get_node_names_and_namespaces()
        except Exception:
            return []

        result = []
        for node_name, namespace in nodes:
            if _is_internal(node_name):
                continue
            full_name = namespace.rstrip('/') + '/' + node_name
            result.append(full_name)
        return sorted(result)

    def _call_service(self, srv_type, srv_name, request, timeout=5.0):
        """Call a ROS2 service synchronously with timeout.
        Uses polling since spin is already running in a separate thread."""
        client = self._node.create_client(srv_type, srv_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(request)
            # Wait for result - spin thread handles callbacks
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if future.done():
                return future.result()
            return None
        finally:
            self._node.destroy_client(client)

    def get_node_parameters(self, node_full_name):
        """Get all parameters for a given node using ListParameters service."""
        if not self._active or not self._node:
            return []

        try:
            srv_name = node_full_name + '/list_parameters'
            req = ListParameters.Request()
            resp = self._call_service(ListParameters, srv_name, req)
            if resp:
                return sorted(resp.result.names)
        except Exception:
            pass
        return []

    _TYPE_NAMES = {
        ParameterType.PARAMETER_BOOL: 'Boolean',
        ParameterType.PARAMETER_INTEGER: 'Integer',
        ParameterType.PARAMETER_DOUBLE: 'Double',
        ParameterType.PARAMETER_STRING: 'String',
        ParameterType.PARAMETER_BYTE_ARRAY: 'Byte array',
        ParameterType.PARAMETER_BOOL_ARRAY: 'Boolean array',
        ParameterType.PARAMETER_INTEGER_ARRAY: 'Integer array',
        ParameterType.PARAMETER_DOUBLE_ARRAY: 'Double array',
        ParameterType.PARAMETER_STRING_ARRAY: 'String array',
    }

    def get_parameter_value(self, node_full_name, param_name):
        """Get a single parameter value using GetParameters service."""
        if not self._active or not self._node:
            return None

        try:
            srv_name = node_full_name + '/get_parameters'
            req = GetParameters.Request()
            req.names = [param_name]
            resp = self._call_service(GetParameters, srv_name, req)
            if resp and resp.values:
                val = resp.values[0]
                ptype = val.type
                type_name = self._TYPE_NAMES.get(ptype, 'Unknown')

                if ptype == ParameterType.PARAMETER_BOOL:
                    value_str = str(val.bool_value)
                elif ptype == ParameterType.PARAMETER_INTEGER:
                    value_str = str(val.integer_value)
                elif ptype == ParameterType.PARAMETER_DOUBLE:
                    value_str = str(val.double_value)
                elif ptype == ParameterType.PARAMETER_STRING:
                    value_str = val.string_value
                elif ptype == ParameterType.PARAMETER_BYTE_ARRAY:
                    value_str = str(list(val.byte_array_value))
                elif ptype == ParameterType.PARAMETER_BOOL_ARRAY:
                    value_str = str(list(val.bool_array_value))
                elif ptype == ParameterType.PARAMETER_INTEGER_ARRAY:
                    value_str = str(list(val.integer_array_value))
                elif ptype == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    value_str = str(list(val.double_array_value))
                elif ptype == ParameterType.PARAMETER_STRING_ARRAY:
                    value_str = str(list(val.string_array_value))
                else:
                    value_str = '(not set)'

                return {'type': type_name, 'value': value_str}
        except Exception:
            pass
        return None

    def set_parameter_value(self, node_full_name, param_name, value):
        """Set a parameter value using SetParameters service."""
        if not self._active or not self._node:
            return {'success': False, 'message': 'Not active'}

        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue

        try:
            # First get current type so we can cast the value correctly
            current = self.get_parameter_value(node_full_name, param_name)
            param_val = ParameterValue()

            if current and current['type'] == 'Integer':
                param_val.type = ParameterType.PARAMETER_INTEGER
                param_val.integer_value = int(value)
            elif current and current['type'] == 'Double':
                param_val.type = ParameterType.PARAMETER_DOUBLE
                param_val.double_value = float(value)
            elif current and current['type'] == 'Boolean':
                param_val.type = ParameterType.PARAMETER_BOOL
                param_val.bool_value = str(value).lower() in ('true', '1', 'yes')
            else:
                param_val.type = ParameterType.PARAMETER_STRING
                param_val.string_value = str(value)

            srv_name = node_full_name + '/set_parameters'
            req = SetParameters.Request()
            param = Parameter()
            param.name = param_name
            param.value = param_val
            req.parameters = [param]

            resp = self._call_service(SetParameters, srv_name, req)
            if resp and resp.results:
                r = resp.results[0]
                if r.successful:
                    return {'success': True, 'message': 'Parameter set successfully'}
                else:
                    return {'success': False, 'message': r.reason or 'Failed'}
            return {'success': False, 'message': 'No response from service'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
