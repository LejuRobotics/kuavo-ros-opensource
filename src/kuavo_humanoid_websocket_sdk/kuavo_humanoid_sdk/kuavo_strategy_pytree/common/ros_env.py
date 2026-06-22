"""
Environment adaptation layer for kuavo_strategy_pytree.
Auto-detects ROS SDK vs WebSocket SDK and provides unified API.

Replace:
    import rospy
    rospy.loginfo(...)
    rospy.Publisher(...)
    rospy.ServiceProxy(...)
    rospy.Subscriber(...)

With:
    from kuavo_humanoid_sdk.kuavo_strategy_pytree.common import ros_env
    ros_env.loginfo(...)
    ros_env.Publisher(...)
    ros_env.ServiceProxy(...)
    ros_env.Subscriber(...)
"""

import time as _time
import sys as _sys

# ========== Environment Detection ==========
try:
    from kuavo_humanoid_sdk.common.logger import SDKLogger
    from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK
    import roslibpy as _roslibpy
    _IS_WS_MODE = True
except ImportError:
    _IS_WS_MODE = False


# ========== WebSocket Client (lazy init) ==========
_ws_client = None


def _get_ws_client():
    global _ws_client
    if _ws_client is None and _IS_WS_MODE:
        _ws_client = WebSocketKuavoSDK().client
    return _ws_client


# ========== Logging ==========
if _IS_WS_MODE:
    def loginfo(msg, *args):
        SDKLogger.info(msg % args if args else msg)

    def logwarn(msg, *args):
        SDKLogger.warning(msg % args if args else msg)

    def logerr(msg, *args):
        SDKLogger.error(msg % args if args else msg)

    def logwarn_throttle(period, msg, *args):
        SDKLogger.warning(msg % args if args else msg)
else:
    import rospy as _rospy
    loginfo = _rospy.loginfo
    logwarn = _rospy.logwarn
    logerr = _rospy.logerr
    logwarn_throttle = _rospy.logwarn_throttle


# ========== Time ==========
if _IS_WS_MODE:
    class _Time:
        """genpy.Time-compatible object for WS mode. Exposes secs/nsecs so
        _msg_to_dict can produce the {"secs": int, "nsecs": int} dict that
        roslibpy/rosbridge expects."""
        __slots__ = ('secs', 'nsecs')

        def __init__(self, secs=0, nsecs=0):
            self.secs = int(secs)
            self.nsecs = int(nsecs)

    def now():
        """Return a _Time object (compatible with genpy.Time) for WS mode."""
        t = _time.time()
        return _Time(secs=int(t), nsecs=int((t - int(t)) * 1e9))

    def get_time():
        return _time.time()

    def sleep(duration):
        _time.sleep(duration)
else:
    def now():
        return _rospy.Time.now()

    def get_time():
        return _rospy.get_time()

    def sleep(duration):
        _rospy.sleep(duration)


# ========== Node Init ==========
if _IS_WS_MODE:
    def init_node(*args, **kwargs):
        """No-op in WebSocket mode."""
        pass

    def is_initialized():
        return True

    class ROSInterruptException(Exception):
        pass
else:
    init_node = _rospy.init_node

    def is_initialized():
        return _rospy.core.is_initialized()

    ROSInterruptException = _rospy.ROSInterruptException


# ========== Exceptions (for except clauses) ==========
if _IS_WS_MODE:
    class ROSException(Exception):
        pass

    class ServiceException(Exception):
        pass
else:
    ROSException = _rospy.ROSException
    ServiceException = _rospy.ServiceException


# ========== Message/Response Conversion Utilities ==========

def _msg_to_dict(msg):
    """Convert a typed ROS message object to a plain dict (for WS mode publishing)."""
    if msg is None:
        return None
    # Handle genpy Time/Duration (secs + nsecs)
    if hasattr(msg, 'secs') and hasattr(msg, 'nsecs'):
        return {'secs': int(msg.secs), 'nsecs': int(msg.nsecs)}
    if hasattr(msg, '__slots__'):
        result = {}
        for slot in msg.__slots__:
            val = getattr(msg, slot)
            result[slot] = _msg_to_dict(val)
        return result
    if isinstance(msg, list):
        return [_msg_to_dict(v) for v in msg]
    if isinstance(msg, tuple):
        return tuple(_msg_to_dict(v) for v in msg)
    return msg


def _wrap_response(data):
    """Recursively wrap a response dict/list so fields are accessible as attributes."""
    if isinstance(data, dict):
        return _DictObj({k: _wrap_response(v) for k, v in data.items()})
    if isinstance(data, list):
        return [_wrap_response(v) for v in data]
    return data


class _DictObj:
    """Dict wrapper: provide attribute-style access for WS mode service/subscriber responses."""

    __slots__ = ('_d',)

    def __init__(self, data):
        object.__setattr__(self, '_d', data if data is not None else {})

    def __getattr__(self, name):
        if name == '_d':
            return object.__getattribute__(self, '_d')
        val = self._d.get(name)
        if isinstance(val, dict):
            return _DictObj(val)
        return val

    def __setattr__(self, name, value):
        self._d[name] = value

    def __repr__(self):
        return f"_DictObj({self._d})"


# ========== Type String Inference ==========

def _infer_ros_type(msg_or_srv_class):
    """Infer the roslibpy type string from a ROS msg/srv class.

    E.g., kuavo_msgs.msg._AprilTagDetectionArray.AprilTagDetectionArray
       -> 'kuavo_msgs/AprilTagDetectionArray'
    """
    module = getattr(msg_or_srv_class, '__module__', '')
    name = getattr(msg_or_srv_class, '__name__', '')
    # module: "kuavo_msgs.msg._AprilTagDetectionArray" or "std_msgs.msg._Bool"
    pkg = module.split('.')[0] if module else 'std_msgs'
    return f"{pkg}/{name}"


# ========== Publisher ==========
if _IS_WS_MODE:
    class Publisher:
        """Publisher compatible with rospy.Publisher interface for WebSocket mode."""

        def __init__(self, topic_name, msg_class, queue_size=10, latch=False, **kwargs):
            ros_type = _infer_ros_type(msg_class)
            self._topic = _roslibpy.Topic(_get_ws_client(), topic_name, ros_type)
            self._topic.advertise()

        def publish(self, msg):
            msg_dict = _msg_to_dict(msg)
            self._topic.publish(_roslibpy.Message(msg_dict))

        def unregister(self):
            self._topic.unadvertise()
else:
    Publisher = _rospy.Publisher


# ========== Subscriber ==========
if _IS_WS_MODE:
    class Subscriber:
        """Subscriber compatible with rospy.Subscriber interface for WebSocket mode."""

        def __init__(self, topic_name, msg_class, callback, queue_size=10, **kwargs):
            ros_type = _infer_ros_type(msg_class)
            self._topic = _roslibpy.Topic(_get_ws_client(), topic_name, ros_type)
            self._callback = callback
            self._topic.subscribe(self._on_message)

        def _on_message(self, msg_dict):
            wrapped = _wrap_response(msg_dict)
            self._callback(wrapped)

        def unregister(self):
            self._topic.unsubscribe()
else:
    Subscriber = _rospy.Subscriber


# ========== Service ==========
if _IS_WS_MODE:
    def wait_for_service(service_name, timeout=5.0):
        """Wait for ROS service to be available. No-op in WS mode."""
        pass

    def ServiceProxy(service_name, srv_class):
        """Create a service proxy callable. Returns _DictObj-wrapped response."""
        ros_type = _infer_ros_type(srv_class)
        service = _roslibpy.Service(_get_ws_client(), service_name, ros_type)

        def _call(request):
            req_dict = _msg_to_dict(request)
            resp_dict = service.call(req_dict)
            return _wrap_response(resp_dict)

        return _call
else:
    def wait_for_service(service_name, timeout=5.0):
        return _rospy.wait_for_service(service_name, timeout=timeout)

    def ServiceProxy(service_name, srv_class):
        return _rospy.ServiceProxy(service_name, srv_class)
