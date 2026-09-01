#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Keep the walking Joy subscription aligned with the active controller."""

import math
import struct
import threading
import time
import uuid
import xmlrpc.client

import rosgraph
import rosgraph.network
import rospy
from kuavo_pico_gmr.srv import AmpJoyRoute, AmpJoyRouteRequest
from kuavo_msgs.msg import ControllerSwitchEvent
from kuavo_msgs.srv import SetJoyTopic, getControllerList
import rospy.impl.tcpros_base as tcpros_base


_IDENTITY_FOUND = "found"
_IDENTITY_ABSENT = "absent"
_IDENTITY_UNKNOWN = "unknown"


class _TimeoutTransport(xmlrpc.client.Transport):
    """Apply a real socket timeout to each ROS XML-RPC operation."""

    def __init__(self, timeout_s):
        super().__init__()
        self._timeout_s = timeout_s

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self._timeout_s
        return connection


class _BoundedServiceTransport(tcpros_base.TCPROSTransport):
    """TCPROS service transport with one wall-clock deadline per call.

    The rospy implementation calls ``socket.setblocking(1)`` inside
    ``receive_once()``, which silently removes the timeout supplied to
    ``connect()``.  Keep the Noetic receive algorithm, but arm the socket with
    the remaining deadline before every potentially blocking receive.
    """

    def __init__(self, protocol, name, timeout_s):
        super().__init__(protocol, name)
        self._timeout_s = timeout_s
        self._deadline = None

    def connect(self, destination_address, destination_port, endpoint_id):
        self._deadline = time.monotonic() + self._timeout_s
        return super().connect(
            destination_address,
            destination_port,
            endpoint_id,
            timeout=self._remaining_timeout(),
        )

    def write_header(self):
        """Send the TCPROS header without Noetic's unbounded writable poll."""

        sock = self.socket
        protocol = self.protocol
        if sock is None or protocol is None:
            return
        self.arm_socket_timeout()
        self.stat_bytes += tcpros_base.write_ros_handshake_header(
            sock,
            protocol.get_header_fields(),
        )

    def read_header(self):
        """Read and validate a TCPROS header under the shared call deadline."""

        sock = self.socket
        if sock is None:
            return
        read_buffer = self.read_buff
        encoded_header = None
        while encoded_header is None:
            self.arm_socket_timeout()
            data = sock.recv(self.protocol.buff_size)
            if not data:
                raise rosgraph.network.ROSHandshakeException(
                    "connection from sender terminated before handshake header "
                    "received. %s bytes were received. Please check sender for "
                    "additional details." % read_buffer.tell()
                )
            read_buffer.write(data)
            buffered_size = read_buffer.tell()
            if buffered_size > 4:
                buffer_value = read_buffer.getvalue()
                (header_size,) = struct.unpack("<I", buffer_value[0:4])
                if buffered_size - 4 >= header_size:
                    encoded_header = buffer_value
                    # Preserve bytes already received after the header.  A fast
                    # service may place response bytes in the same TCP packet.
                    leftovers = buffer_value[header_size + 4 :]
                    read_buffer.truncate(len(leftovers))
                    read_buffer.seek(0)
                    read_buffer.write(leftovers)

        self._validate_header(
            rosgraph.network.decode_ros_handshake_header(encoded_header)
        )

    def arm_socket_timeout(self):
        if self.socket is None:
            raise tcpros_base.TransportException("connection not initialized")
        self.socket.settimeout(self._remaining_timeout())

    def _remaining_timeout(self):
        if self._deadline is None:
            return self._timeout_s
        remaining = self._deadline - time.monotonic()
        if remaining <= 0.0:
            raise tcpros_base.TransportException(
                "service call exceeded %.3fs timeout" % self._timeout_s
            )
        return remaining

    def receive_once(self):
        sock = self.socket
        if sock is None:
            raise tcpros_base.TransportException("connection not initialized")
        read_buffer = self.read_buff
        messages = []
        protocol = self.protocol
        try:
            while not messages and not self.done and not tcpros_base.is_shutdown():
                self.arm_socket_timeout()
                if read_buffer.tell() >= 4:
                    protocol.read_messages(read_buffer, messages, sock)
                if not messages:
                    self.arm_socket_timeout()
                    self.stat_bytes += tcpros_base.recv_buff(
                        sock,
                        read_buffer,
                        protocol.buff_size,
                    )

            self.stat_num_msg += len(messages)
            for message in messages:
                message._connection_header = self.header
            if self.is_latched and messages:
                self.latch = messages[-1]
            return messages
        except tcpros_base.DeserializationError as exception:
            raise tcpros_base.TransportException(
                "receive_once[%s]: DeserializationError %s"
                % (self.name, exception)
            )
        except (tcpros_base.TransportTerminated, tcpros_base.ServiceException):
            raise
        except Exception as exception:
            raise tcpros_base.TransportException(
                "receive_once[%s]: unexpected error %s"
                % (self.name, exception)
            )


class PicoJoyTopicManager:
    def __init__(self):
        rospy.init_node("pico_gmr_joy_topic_manager")

        self._event_topic = rospy.get_param(
            "~controller_switch_event_topic",
            "/humanoid_controller/controller_switch_event",
        )
        self._controller_list_service = rospy.get_param(
            "~controller_list_service",
            "/humanoid_controller/get_controller_list",
        )
        self._set_joy_topic_service = rospy.get_param(
            "~set_joy_topic_service",
            "/set_joy_topic",
        )
        self._prepare_amp_route_service = rospy.get_param(
            "~prepare_amp_route_service",
            "/pico_gmr_joy_converter/prepare_amp_route",
        )
        self._route_subscriber_node = rospy.resolve_name(
            rospy.get_param(
                "~route_subscriber_node",
                "/humanoid_joy_control_auto_gait_with_vel",
            )
        )
        self._pico_joy_topic = rospy.get_param(
            "~pico_joy_topic",
            "/pico/joy_converted",
        )
        self._default_joy_topic = rospy.get_param("~default_joy_topic", "/joy")
        self._retry_period_s = self._positive_finite_param(
            "~retry_period_s",
            1.0,
        )
        self._master_probe_timeout_s = self._positive_finite_param(
            "~master_probe_timeout_s",
            0.5,
        )
        self._service_call_timeout_s = self._positive_finite_param(
            "~service_call_timeout_s",
            1.0,
        )
        self._route_ready_timeout_s = self._positive_finite_param(
            "~route_ready_timeout_s",
            1.5,
        )

        configured_amp_names = rospy.get_param(
            "~amp_controller_names",
            [
                "amp_controller",
                "amp_wild_controller",
                "amp_run_controller",
                "amp_hand_controller",
            ],
        )
        if isinstance(configured_amp_names, str):
            configured_amp_names = [configured_amp_names]
        if not isinstance(configured_amp_names, list):
            raise ValueError("~amp_controller_names must be a string or list of strings")
        self._amp_controller_names = {
            self._normalize_controller_name(name)
            for name in configured_amp_names
            if isinstance(name, str) and name.strip()
        }
        if not self._amp_controller_names:
            raise ValueError("~amp_controller_names must contain at least one name")

        # All controller reconciliation is serialized in one worker.  ROS
        # subscriber callbacks only update desired state and wake this worker.
        self._state_cv = threading.Condition()
        self._stopping = False
        self._wake_revision = 0
        self._event_revision = 0
        self._controller_known = False
        self._desired_topic = None
        self._applied_topic = None
        self._desired_route_revision = 0
        self._desired_route_context = {
            "reason": AmpJoyRouteRequest.REASON_CONTROLLER_QUERY,
            "event_revision": 0,
            "event_stamp": rospy.Time(),
            "from_controller": "",
            "to_controller": "",
        }
        self._manager_epoch = uuid.uuid4().hex
        self._next_route_generation = 0
        self._route_transaction = None

        # UNKNOWN is deliberately different from ABSENT.  A transient Master
        # timeout must not erase a previously verified provider identity.
        self._set_topic_service_identity_status = _IDENTITY_UNKNOWN
        self._set_topic_service_identity = None
        self._set_topic_service_generation = 0
        self._prepare_route_service_identity_status = _IDENTITY_UNKNOWN
        self._prepare_route_service_identity = None
        self._prepare_route_service_generation = 0
        self._controller_list_service_identity_status = _IDENTITY_UNKNOWN
        self._controller_list_service_identity = None
        self._controller_list_service_generation = 0

        self._master_uri = rosgraph.get_master_uri()
        self._set_joy_topic = rospy.ServiceProxy(
            self._set_joy_topic_service,
            SetJoyTopic,
        )
        self._prepare_amp_route = rospy.ServiceProxy(
            self._prepare_amp_route_service,
            AmpJoyRoute,
        )
        self._get_controller_list = rospy.ServiceProxy(
            self._controller_list_service,
            getControllerList,
        )
        self._event_subscriber = rospy.Subscriber(
            self._event_topic,
            ControllerSwitchEvent,
            self._on_controller_switch,
            queue_size=10,
        )

        self._worker = threading.Thread(
            target=self._reconcile_loop,
            name="pico-gmr-joy-topic-reconciler",
            daemon=True,
        )
        rospy.on_shutdown(self._request_stop)
        self._worker.start()

        rospy.loginfo(
            "Pico joy topic manager epoch=%s: AMP=%s, other=%s, AMP controllers=%s",
            self._manager_epoch,
            self._pico_joy_topic,
            self._default_joy_topic,
            sorted(self._amp_controller_names),
        )

    @staticmethod
    def _positive_finite_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("%s must be finite and > 0" % name)
        return value

    @staticmethod
    def _normalize_controller_name(name):
        return str(name).strip().lower()

    def _topic_for_controller(self, controller_name):
        normalized = self._normalize_controller_name(controller_name)
        if normalized in self._amp_controller_names:
            return self._pico_joy_topic
        return self._default_joy_topic

    def _on_controller_switch(self, message):
        controller_name = str(message.to_controller).strip()
        if not controller_name:
            rospy.logwarn("Ignore controller switch with an empty destination")
            return

        desired_topic = self._topic_for_controller(controller_name)
        with self._state_cv:
            self._event_revision += 1
            changed = desired_topic != self._desired_topic
            if changed:
                self._desired_route_revision += 1
                # A discarded AMP transaction may already have an in-flight
                # /set_joy_topic request.  Its service callback can mutate the
                # real AutoGait subscription even though the stale response is
                # ignored below.  Forget the cached route so the new desired
                # controller always performs an authoritative setter call.
                if self._route_transaction is not None:
                    self._applied_topic = None
                self._route_transaction = None
            self._desired_topic = desired_topic
            self._desired_route_context = {
                "reason": AmpJoyRouteRequest.REASON_CONTROLLER_EVENT,
                "event_revision": self._event_revision,
                "event_stamp": message.header.stamp,
                "from_controller": str(message.from_controller),
                "to_controller": controller_name,
            }
            self._controller_known = True
            self._wake_revision += 1
            self._state_cv.notify()

        rospy.loginfo(
            "Controller switch: %s -> %s",
            message.from_controller,
            message.to_controller,
        )
        if changed:
            rospy.loginfo(
                "Controller %s from switch event selects Joy topic %s",
                controller_name,
                desired_topic,
            )

    def _reconcile_loop(self):
        observed_wake_revision = -1
        next_identity_refresh = 0.0

        while True:
            with self._state_cv:
                while not self._stopping:
                    now = time.monotonic()
                    if (
                        self._wake_revision != observed_wake_revision
                        or now >= next_identity_refresh
                    ):
                        break
                    self._state_cv.wait(
                        timeout=max(0.0, next_identity_refresh - now)
                    )
                if self._stopping:
                    return
                # Snapshot before doing work.  Any event arriving during an RPC
                # increments the revision and forces another pass before sleep.
                observed_wake_revision = self._wake_revision

            try:
                if time.monotonic() >= next_identity_refresh:
                    self._refresh_service_registrations()
                    self._audit_joy_subscription()
                    next_identity_refresh = time.monotonic() + self._retry_period_s

                self._query_current_controller()
                self._apply_desired_topic()
            except Exception as exception:
                # An unexpected provider response must not permanently kill the
                # sole reconciler.  Normal transport failures are handled closer
                # to their call sites and will be retried on the next period.
                rospy.logerr_throttle(
                    5.0,
                    "Pico Joy topic reconciliation failed: %s" % exception,
                )
                next_identity_refresh = time.monotonic() + self._retry_period_s

    def _query_current_controller(self):
        with self._state_cv:
            if self._controller_known:
                return
            if (
                self._controller_list_service_identity_status != _IDENTITY_FOUND
                or self._controller_list_service_identity is None
            ):
                return
            event_revision = self._event_revision
            service_generation = self._controller_list_service_generation
            service_uri = self._controller_list_service_identity[0]

        request = self._get_controller_list.request_class()
        try:
            response = self._call_service_bounded(
                self._get_controller_list,
                service_uri,
                request,
            )
        except Exception as exception:
            rospy.logwarn_throttle(
                5.0,
                "Controller list service unavailable: %s" % exception,
            )
            return

        if not response.success:
            rospy.logwarn_throttle(
                5.0,
                "Controller list query rejected: %s" % response.message,
            )
            return

        controller_name = str(response.current_controller).strip()
        if not controller_name:
            rospy.logwarn_throttle(
                5.0,
                "Controller list service returned an empty current controller",
            )
            return
        desired_topic = self._topic_for_controller(controller_name)

        with self._state_cv:
            # Reject both an event that arrived during the call and a response
            # from a provider generation that is no longer authoritative.
            if (
                self._event_revision != event_revision
                or self._controller_list_service_generation != service_generation
                or self._controller_list_service_identity_status != _IDENTITY_FOUND
            ):
                return
            changed = desired_topic != self._desired_topic
            if changed:
                self._desired_route_revision += 1
                if self._route_transaction is not None:
                    self._applied_topic = None
                self._route_transaction = None
            self._desired_topic = desired_topic
            self._desired_route_context = {
                "reason": AmpJoyRouteRequest.REASON_CONTROLLER_QUERY,
                "event_revision": self._event_revision,
                "event_stamp": rospy.Time(),
                "from_controller": "",
                "to_controller": controller_name,
            }
            self._controller_known = True

        if changed:
            rospy.loginfo(
                "Controller %s from controller query selects Joy topic %s",
                controller_name,
                desired_topic,
            )

    def _apply_desired_topic(self):
        # Chase controller events that arrive while a bounded service call is
        # in flight.  AMP routing is a transaction: prepare the converter once,
        # ask AutoGait to subscribe, then require a same-token target-connection
        # ACK before considering the route applied.
        for _attempt in range(8):
            with self._state_cv:
                desired_topic = self._desired_topic
            if desired_topic is None:
                return

            if desired_topic == self._pico_joy_topic:
                result = self._reconcile_amp_route()
            else:
                result = self._reconcile_default_route()
            if result != "desired_changed":
                return

        rospy.logwarn(
            "Joy topic kept changing while applying; retry latest state next pass"
        )

    def _new_route_transaction_locked(self):
        self._next_route_generation += 1
        context = dict(self._desired_route_context)
        transaction = {
            "manager_epoch": self._manager_epoch,
            "route_generation": self._next_route_generation,
            "desired_revision": self._desired_route_revision,
            "set_generation": self._set_topic_service_generation,
            "set_service_uri": self._set_topic_service_identity[0],
            "prepare_generation": self._prepare_route_service_generation,
            "prepare_service_uri": self._prepare_route_service_identity[0],
            "reason": context["reason"],
            "event_revision": context["event_revision"],
            "event_stamp": context["event_stamp"],
            "from_controller": context["from_controller"],
            "to_controller": context["to_controller"],
            # Unknown/fused applied state first goes through a confirmed raw
            # route.  Besides being a safe fallback, this guarantees that any
            # old converted-topic TCP connection is closing before PREPARE;
            # only a subsequent /set_joy_topic(fused) connection may ACK the
            # new generation.
            "phase": (
                "new"
                if self._applied_topic == self._default_joy_topic
                else "preflight_raw"
            ),
            "ready_deadline": None,
        }
        self._route_transaction = transaction
        return transaction

    def _route_transaction_is_current_locked(self, transaction):
        return (
            self._route_transaction is transaction
            and self._desired_topic == self._pico_joy_topic
            and self._desired_route_revision == transaction["desired_revision"]
            and self._set_topic_service_generation == transaction["set_generation"]
            and self._prepare_route_service_generation
            == transaction["prepare_generation"]
            and self._set_topic_service_identity_status == _IDENTITY_FOUND
            and self._prepare_route_service_identity_status == _IDENTITY_FOUND
        )

    @staticmethod
    def _route_response_matches(transaction, response):
        return (
            response.manager_epoch == transaction["manager_epoch"]
            and response.route_generation == transaction["route_generation"]
        )

    def _make_route_request(self, transaction, action):
        request = AmpJoyRouteRequest()
        request.action = action
        request.manager_epoch = transaction["manager_epoch"]
        request.route_generation = transaction["route_generation"]
        request.reason = transaction["reason"]
        request.controller_event_revision = transaction["event_revision"]
        request.controller_event_stamp = transaction["event_stamp"]
        request.from_controller = transaction["from_controller"]
        request.to_controller = transaction["to_controller"]
        return request

    def _call_route_transaction(self, transaction, action):
        request = self._make_route_request(transaction, action)
        return self._call_service_bounded(
            self._prepare_amp_route,
            transaction["prepare_service_uri"],
            request,
        )

    def _reconcile_amp_route(self):
        with self._state_cv:
            if self._desired_topic != self._pico_joy_topic:
                return "desired_changed"
            if (
                self._set_topic_service_identity_status != _IDENTITY_FOUND
                or self._set_topic_service_identity is None
            ):
                return "wait"
            if (
                self._prepare_route_service_identity_status != _IDENTITY_FOUND
                or self._prepare_route_service_identity is None
            ):
                if self._applied_topic == self._default_joy_topic:
                    return "wait"
                service_generation = self._set_topic_service_generation
                service_uri = self._set_topic_service_identity[0]
                route_unavailable = True
            else:
                route_unavailable = False

            if route_unavailable:
                transaction = None
            else:
                transaction = self._route_transaction
                if (
                    transaction is None
                    or transaction["desired_revision"] != self._desired_route_revision
                    or transaction["set_generation"]
                    != self._set_topic_service_generation
                    or transaction["prepare_generation"]
                    != self._prepare_route_service_generation
                ):
                    transaction = self._new_route_transaction_locked()

        if route_unavailable:
            return self._fallback_when_converter_unavailable(
                service_generation,
                service_uri,
            )

        if transaction["phase"] == "preflight_raw":
            request = self._set_joy_topic.request_class()
            request.topic_name = self._default_joy_topic
            try:
                response = self._call_service_bounded(
                    self._set_joy_topic,
                    transaction["set_service_uri"],
                    request,
                )
            except Exception as exception:
                rospy.logwarn_throttle(
                    5.0,
                    "AMP route raw-Joy preflight unavailable: %s" % exception,
                )
                return "wait"
            if not response.success:
                rospy.logwarn_throttle(
                    5.0,
                    "AMP route raw-Joy preflight rejected: %s" % response.message,
                )
                return "wait"
            with self._state_cv:
                if not self._route_transaction_is_current_locked(transaction):
                    return "desired_changed"
                self._applied_topic = self._default_joy_topic
                transaction["phase"] = "new"

        if transaction["phase"] == "new":
            try:
                response = self._call_route_transaction(
                    transaction,
                    AmpJoyRouteRequest.ACTION_PREPARE,
                )
            except Exception as exception:
                rospy.logwarn_throttle(
                    5.0,
                    "AMP Joy route prepare service unavailable: %s" % exception,
                )
                return "wait"

            if not response.success:
                rospy.logwarn_throttle(
                    5.0,
                    "AMP Joy route prepare rejected: %s" % response.message,
                )
                return "wait"
            if not self._route_response_matches(transaction, response):
                rospy.logwarn("Ignore mismatched AMP route prepare response")
                return "wait"

            with self._state_cv:
                if not self._route_transaction_is_current_locked(transaction):
                    return "desired_changed"
                transaction["phase"] = "prepared"

        if transaction["phase"] == "prepared":
            request = self._set_joy_topic.request_class()
            request.topic_name = self._pico_joy_topic
            try:
                response = self._call_service_bounded(
                    self._set_joy_topic,
                    transaction["set_service_uri"],
                    request,
                )
            except Exception as exception:
                # The server may have executed before the bounded client timed
                # out.  Do not immediately repeat the setter: that would tear
                # down a connection which may already be establishing this
                # token's neutral/input-generation barrier.  First wait for
                # the same-token ready ACK; on timeout the normal raw-/joy
                # fallback closes the ambiguous route before a fresh retry.
                rospy.logwarn_throttle(
                    5.0,
                    "Set AMP Joy topic service unavailable: %s" % exception,
                )
                with self._state_cv:
                    if not self._route_transaction_is_current_locked(transaction):
                        return "desired_changed"
                    transaction["phase"] = "waiting_ready"
                    transaction["ready_deadline"] = (
                        time.monotonic() + self._route_ready_timeout_s
                    )
                response = None

            if response is not None and not response.success:
                rospy.logwarn_throttle(
                    5.0,
                    "Set AMP Joy topic rejected: %s" % response.message,
                )
                return self._fallback_amp_route(transaction)

            with self._state_cv:
                if not self._route_transaction_is_current_locked(transaction):
                    return "desired_changed"
                transaction["phase"] = "waiting_ready"
                transaction["ready_deadline"] = (
                    time.monotonic() + self._route_ready_timeout_s
                )

        try:
            route_status = self._call_route_transaction(
                transaction,
                AmpJoyRouteRequest.ACTION_STATUS,
            )
        except Exception as exception:
            rospy.logwarn_throttle(
                5.0,
                "AMP Joy route status unavailable: %s" % exception,
            )
            route_status = None

        ready = (
            route_status is not None
            and self._route_response_matches(transaction, route_status)
            and route_status.success
            and route_status.ready
            and route_status.target_connected
            and route_status.local_barrier_established
        )
        if ready:
            with self._state_cv:
                if not self._route_transaction_is_current_locked(transaction):
                    return "desired_changed"
                was_applied = self._applied_topic == self._pico_joy_topic
                transaction["phase"] = "ready"
                transaction["ready_deadline"] = None
                self._applied_topic = self._pico_joy_topic
            if not was_applied:
                rospy.loginfo(
                    "Joy topic switched to %s with route generation %d ready",
                    self._pico_joy_topic,
                    transaction["route_generation"],
                )
            return "done"

        with self._state_cv:
            if not self._route_transaction_is_current_locked(transaction):
                return "desired_changed"
            if transaction["ready_deadline"] is None:
                transaction["phase"] = "waiting_ready"
                transaction["ready_deadline"] = (
                    time.monotonic() + self._route_ready_timeout_s
                )
                self._applied_topic = None
            deadline_expired = time.monotonic() >= transaction["ready_deadline"]

        if not deadline_expired:
            return "wait"

        rospy.logerr(
            "AMP Joy route generation %d did not become ready within %.3fs; "
            "falling back to %s",
            transaction["route_generation"],
            self._route_ready_timeout_s,
            self._default_joy_topic,
        )
        return self._fallback_amp_route(transaction)

    def _fallback_when_converter_unavailable(
        self,
        service_generation,
        service_uri,
    ):
        """Keep AMP controllable through raw /joy while its converter is absent."""

        request = self._set_joy_topic.request_class()
        request.topic_name = self._default_joy_topic
        try:
            response = self._call_service_bounded(
                self._set_joy_topic,
                service_uri,
                request,
            )
        except Exception as exception:
            rospy.logwarn_throttle(
                5.0,
                "Cannot fall back to default Joy while AMP converter is "
                "unavailable: %s" % exception,
            )
            return "wait"
        if not response.success:
            rospy.logwarn_throttle(
                5.0,
                "Default Joy fallback rejected while AMP converter is "
                "unavailable: %s" % response.message,
            )
            return "wait"

        with self._state_cv:
            if (
                self._desired_topic != self._pico_joy_topic
                or self._set_topic_service_generation != service_generation
                or self._set_topic_service_identity_status != _IDENTITY_FOUND
            ):
                return "desired_changed"
            self._applied_topic = self._default_joy_topic

        rospy.logerr(
            "AMP Joy converter route is unavailable; AutoGait fell back to %s. "
            "Pico walking is disabled until the converter route recovers.",
            self._default_joy_topic,
        )
        return "wait"

    def _fallback_amp_route(self, transaction):
        request = self._set_joy_topic.request_class()
        request.topic_name = self._default_joy_topic
        try:
            response = self._call_service_bounded(
                self._set_joy_topic,
                transaction["set_service_uri"],
                request,
            )
        except Exception as exception:
            rospy.logwarn_throttle(
                5.0,
                "Failed to restore default Joy after AMP route timeout: %s"
                % exception,
            )
            return "wait"
        if not response.success:
            rospy.logwarn_throttle(
                5.0,
                "Default Joy fallback rejected: %s" % response.message,
            )
            return "wait"

        with self._state_cv:
            if self._set_topic_service_generation != transaction["set_generation"]:
                return "desired_changed"
            self._applied_topic = self._default_joy_topic
            if self._route_transaction is transaction:
                self._route_transaction = None
        self._abort_route_transaction(transaction)
        return "wait"

    def _reconcile_default_route(self):
        with self._state_cv:
            if self._desired_topic == self._pico_joy_topic:
                return "desired_changed"
            if (
                self._set_topic_service_identity_status != _IDENTITY_FOUND
                or self._set_topic_service_identity is None
            ):
                return "wait"
            transaction = self._route_transaction
            if self._applied_topic == self._default_joy_topic and transaction is None:
                return "done"
            service_generation = self._set_topic_service_generation
            service_uri = self._set_topic_service_identity[0]

        # Confirm raw /joy before aborting a prepared converter transaction.
        # A late/unknown AMP set response can otherwise leave AutoGait on the
        # frozen converted topic while the converter has already unfrozen.
        request = self._set_joy_topic.request_class()
        request.topic_name = self._default_joy_topic
        try:
            response = self._call_service_bounded(
                self._set_joy_topic,
                service_uri,
                request,
            )
        except Exception as exception:
            rospy.logwarn_throttle(
                5.0,
                "Set default Joy topic service unavailable: %s" % exception,
            )
            return "wait"
        if not response.success:
            rospy.logwarn_throttle(
                5.0,
                "Set default Joy topic rejected: %s" % response.message,
            )
            return "wait"

        with self._state_cv:
            if (
                self._set_topic_service_generation != service_generation
                or self._set_topic_service_identity_status != _IDENTITY_FOUND
            ):
                return "desired_changed"
            self._applied_topic = self._default_joy_topic
            latest_desired_topic = self._desired_topic
            if self._route_transaction is transaction:
                self._route_transaction = None

        if transaction is not None:
            self._abort_route_transaction(transaction)
        rospy.loginfo("Joy topic switched to %s", self._default_joy_topic)
        return (
            "done"
            if latest_desired_topic != self._pico_joy_topic
            else "desired_changed"
        )

    def _abort_route_transaction(self, transaction):
        try:
            response = self._call_route_transaction(
                transaction,
                AmpJoyRouteRequest.ACTION_ABORT,
            )
            if not self._route_response_matches(transaction, response):
                rospy.logwarn("Ignore mismatched AMP route abort response")
            elif not response.success:
                rospy.logwarn_throttle(
                    5.0,
                    "AMP Joy route abort rejected: %s" % response.message,
                )
        except Exception as exception:
            # The target has already been confirmed on raw /joy.  A failed
            # cleanup cannot compromise that route; a future PREPARE with a new
            # token supersedes this converter state.
            rospy.logwarn_throttle(
                5.0,
                "AMP Joy route abort service unavailable: %s" % exception,
            )

    def _call_service_bounded(self, proxy, service_uri, request):
        """Call a rospy service without an unbounded Master or TCP wait.

        ServiceProxy has no call-timeout option.  Supplying an already connected
        TCPROSTransport makes it skip its own unbounded Master lookup, while the
        socket timeout bounds connect, header exchange and response reads.  All
        service calls use the single reconciler thread, so this does not create
        one potentially leaked helper thread per retry.
        """

        destination_address, destination_port = rospy.core.parse_rosrpc_uri(
            service_uri
        )
        transport = _BoundedServiceTransport(
            proxy.protocol,
            proxy.resolved_name,
            self._service_call_timeout_s,
        )
        transport.buff_size = proxy.buff_size
        try:
            transport.connect(
                destination_address,
                destination_port,
                service_uri,
            )
            # connect() bounds the TCPROS header exchange.  Re-arm with the
            # remaining wall-clock deadline immediately before request send.
            transport.arm_socket_timeout()
            proxy.transport = transport
            return proxy(request)
        finally:
            # Non-persistent ServiceProxy normally closes this itself, but send
            # failures occur before rospy's receive-side cleanup block.
            if proxy.transport is transport:
                proxy.transport = None
            transport.close()

    def _master_rpc(self, method_name, *arguments):
        # A fresh proxy/transport avoids sharing httplib connection state and
        # applies the timeout to every getSystemState/lookup* request.
        with xmlrpc.client.ServerProxy(
            self._master_uri,
            transport=_TimeoutTransport(self._master_probe_timeout_s),
            allow_none=True,
        ) as master:
            method = getattr(master, method_name)
            return method(rospy.get_name(), *arguments)

    def _lookup_service_identity(self, service_name):
        resolved_service_name = rospy.resolve_name(service_name)
        try:
            code, _message, state = self._master_rpc("getSystemState")
            if code != 1:
                return _IDENTITY_UNKNOWN, None

            provider_names = []
            for registered_service, providers in state[2]:
                if registered_service == resolved_service_name:
                    provider_names.extend(providers)
            provider_names = sorted(set(provider_names))
            if not provider_names:
                return _IDENTITY_ABSENT, None

            code, _message, service_uri = self._master_rpc(
                "lookupService",
                resolved_service_name,
            )
            if code != 1 or not service_uri:
                # Registration existed in the same probe, so a failed lookup is
                # a race or communication error, not evidence of absence.
                return _IDENTITY_UNKNOWN, None

            provider_identities = []
            for provider_name in provider_names:
                code, _message, node_uri = self._master_rpc(
                    "lookupNode",
                    provider_name,
                )
                if code != 1 or not node_uri:
                    return _IDENTITY_UNKNOWN, None
                with xmlrpc.client.ServerProxy(
                    node_uri,
                    transport=_TimeoutTransport(self._master_probe_timeout_s),
                    allow_none=True,
                ) as provider:
                    code, _message, provider_pid = provider.getPid(rospy.get_name())
                if code != 1:
                    return _IDENTITY_UNKNOWN, None
                provider_identities.append(
                    (provider_name, node_uri, int(provider_pid))
                )
        except Exception as exception:  # XML-RPC/socket errors are not ROSException.
            rospy.logwarn_throttle(
                5.0,
                "ROS registration lookup failed for %s: %s"
                % (resolved_service_name, exception),
            )
            return _IDENTITY_UNKNOWN, None

        # lookupService's ROSRPC URI is essential: a node can recreate a service
        # at a new TCP port without changing either its XML-RPC URI or PID.
        return _IDENTITY_FOUND, (service_uri, tuple(provider_identities))

    def _lookup_joy_subscriptions(self):
        """Read the target node's actual raw/fused Joy subscriptions.

        SetJoyTopic has no getter and a client-side timeout cannot cancel a
        request that the server has already accepted.  The ROS node XML-RPC
        API is therefore the authoritative read-back used to repair a late
        service mutation or an out-of-band caller of /set_joy_topic.
        """

        try:
            code, _message, node_uri = self._master_rpc(
                "lookupNode",
                self._route_subscriber_node,
            )
            if code != 1 or not node_uri:
                return _IDENTITY_ABSENT, set()
            with xmlrpc.client.ServerProxy(
                node_uri,
                transport=_TimeoutTransport(self._master_probe_timeout_s),
                allow_none=True,
            ) as node:
                code, _message, subscriptions = node.getSubscriptions(
                    rospy.get_name()
                )
            if code != 1:
                return _IDENTITY_UNKNOWN, set()
        except Exception as exception:
            rospy.logwarn_throttle(
                5.0,
                "ROS subscription lookup failed for %s: %s"
                % (self._route_subscriber_node, exception),
            )
            return _IDENTITY_UNKNOWN, set()

        expected_topics = {self._default_joy_topic, self._pico_joy_topic}
        actual_topics = {
            str(subscription[0])
            for subscription in subscriptions
            if isinstance(subscription, (list, tuple))
            and len(subscription) >= 2
            and str(subscription[0]) in expected_topics
            and str(subscription[1]) == "sensor_msgs/Joy"
        }
        return _IDENTITY_FOUND, actual_topics

    def _audit_joy_subscription(self):
        status, actual_topics = self._lookup_joy_subscriptions()
        if status == _IDENTITY_UNKNOWN:
            return

        with self._state_cv:
            applied_topic = self._applied_topic
            transaction = self._route_transaction
            # PREPARE/set/READY transitions intentionally pass through raw,
            # disconnected, and fused graph states.  Their tokenized STATUS
            # ACK is stronger than a point-in-time snapshot.  A transaction in
            # phase "new", however, has not called PREPARE yet and requires a
            # confirmed raw route; auditing it prevents a late/out-of-band
            # fused setter from leaving PREPARE stuck behind an old connection.
            if transaction is not None and transaction["phase"] in (
                "preflight_raw",
                "prepared",
                "waiting_ready",
            ):
                return
            if applied_topic not in (self._default_joy_topic, self._pico_joy_topic):
                return
            if actual_topics == {applied_topic}:
                return

            self._applied_topic = None
            if transaction is not None:
                self._route_transaction = None
            self._wake_revision += 1
            self._state_cv.notify()

        rospy.logwarn_throttle(
            5.0,
            "AutoGait Joy subscription drift detected: cached=%s, actual=%s; "
            "reapplying the controller-selected route"
            % (applied_topic, sorted(actual_topics)),
        )

    def _refresh_service_registrations(self):
        set_topic_probe = self._lookup_service_identity(
            self._set_joy_topic_service
        )
        prepare_route_probe = self._lookup_service_identity(
            self._prepare_amp_route_service
        )
        with self._state_cv:
            controller_probe_event_revision = self._event_revision
        controller_list_probe = self._lookup_service_identity(
            self._controller_list_service
        )

        set_topic_changed = self._commit_identity_probe(
            "set_topic",
            *set_topic_probe,
        )
        prepare_route_changed = self._commit_identity_probe(
            "prepare_route",
            *prepare_route_probe,
        )
        controller_list_changed = self._commit_identity_probe(
            "controller_list",
            *controller_list_probe,
            controller_probe_event_revision,
        )

        if set_topic_changed:
            rospy.logwarn(
                "Set Joy topic service registration changed; reapplying desired topic"
            )
        if prepare_route_changed:
            rospy.logwarn(
                "AMP route prepare service registration changed; "
                "reconciling the fused route when needed"
            )
        if controller_list_changed:
            rospy.logwarn(
                "Controller list service registration changed; querying controller again"
            )

    def _commit_identity_probe(
        self,
        service_kind,
        status,
        identity,
        controller_probe_event_revision=None,
    ):
        if status == _IDENTITY_UNKNOWN:
            # Preserve the last verified status and identity on probe failure.
            return False

        with self._state_cv:
            if service_kind == "set_topic":
                old_status = self._set_topic_service_identity_status
                old_identity = self._set_topic_service_identity
                changed = old_status != status or old_identity != identity
                if not changed:
                    return False
                self._set_topic_service_identity_status = status
                self._set_topic_service_identity = identity
                self._set_topic_service_generation += 1
                self._applied_topic = None
                if (
                    old_status != _IDENTITY_UNKNOWN
                    and self._desired_topic == self._pico_joy_topic
                ):
                    self._desired_route_revision += 1
                    self._desired_route_context = {
                        "reason": AmpJoyRouteRequest.REASON_SET_TOPIC_RECOVERY,
                        "event_revision": self._event_revision,
                        "event_stamp": rospy.Time(),
                        "from_controller": "",
                        "to_controller": "",
                    }
                    self._route_transaction = None
            elif service_kind == "prepare_route":
                old_status = self._prepare_route_service_identity_status
                old_identity = self._prepare_route_service_identity
                changed = old_status != status or old_identity != identity
                if not changed:
                    return False
                self._prepare_route_service_identity_status = status
                self._prepare_route_service_identity = identity
                self._prepare_route_service_generation += 1
                # A restarted converter has lost its layout/edge guards.  Only
                # invalidate a route that uses the fused AMP topic; in a
                # non-AMP mode the raw /joy subscription is independent of
                # this service and must not be needlessly torn down/recreated.
                if (
                    old_status != _IDENTITY_UNKNOWN
                    and (
                        self._desired_topic == self._pico_joy_topic
                        or self._applied_topic == self._pico_joy_topic
                    )
                ):
                    self._applied_topic = None
                    self._desired_route_revision += 1
                    self._desired_route_context = {
                        "reason": AmpJoyRouteRequest.REASON_CONVERTER_RECOVERY,
                        "event_revision": self._event_revision,
                        "event_stamp": rospy.Time(),
                        "from_controller": "",
                        "to_controller": "",
                    }
                    self._route_transaction = None
            elif service_kind == "controller_list":
                old_status = self._controller_list_service_identity_status
                old_identity = self._controller_list_service_identity
                changed = old_status != status or old_identity != identity
                if not changed:
                    return False
                self._controller_list_service_identity_status = status
                self._controller_list_service_identity = identity
                self._controller_list_service_generation += 1
                # A switch event that arrived while the identity probe was in
                # flight is newer than that probe and remains authoritative.
                if self._event_revision == controller_probe_event_revision:
                    self._controller_known = False
            else:
                raise ValueError("Unknown service kind: %s" % service_kind)

        # Do not report the first successful inventory as a provider change.
        return old_status != _IDENTITY_UNKNOWN

    def _request_stop(self):
        with self._state_cv:
            self._stopping = True
            self._state_cv.notify_all()

    def shutdown(self):
        self._request_stop()
        if threading.current_thread() is self._worker:
            return
        # Every I/O operation is bounded; this is only a grace period for the
        # current reconciliation pass.  The daemon flag remains a final guard.
        join_timeout = (
            8.0 * self._master_probe_timeout_s
            + 4.0 * self._service_call_timeout_s
            + 1.0
        )
        self._worker.join(timeout=join_timeout)


if __name__ == "__main__":
    manager = None
    try:
        manager = PicoJoyTopicManager()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as exception:
        if not rospy.is_shutdown():
            rospy.logfatal("Pico joy topic manager stopped: %s", exception)
    finally:
        if manager is not None:
            manager.shutdown()
