#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Keep the walking Joy subscription aligned with the active controller."""

import math
import struct
import threading
import time
import xmlrpc.client

import rosgraph
import rosgraph.network
import rospy
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
        rospy.init_node("pico_joy_topic_manager")

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

        # UNKNOWN is deliberately different from ABSENT.  A transient Master
        # timeout must not erase a previously verified provider identity.
        self._set_topic_service_identity_status = _IDENTITY_UNKNOWN
        self._set_topic_service_identity = None
        self._set_topic_service_generation = 0
        self._controller_list_service_identity_status = _IDENTITY_UNKNOWN
        self._controller_list_service_identity = None
        self._controller_list_service_generation = 0

        self._master_uri = rosgraph.get_master_uri()
        self._set_joy_topic = rospy.ServiceProxy(
            self._set_joy_topic_service,
            SetJoyTopic,
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
            name="pico-joy-topic-reconciler",
            daemon=True,
        )
        rospy.on_shutdown(self._request_stop)
        self._worker.start()

        rospy.loginfo(
            "Pico joy topic manager: AMP=%s, other=%s, AMP controllers=%s",
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
            self._desired_topic = desired_topic
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
            self._desired_topic = desired_topic
            self._controller_known = True

        if changed:
            rospy.loginfo(
                "Controller %s from controller query selects Joy topic %s",
                controller_name,
                desired_topic,
            )

    def _apply_desired_topic(self):
        # Chase controller events that arrive while a service call is in flight.
        # The cap protects the worker if another source is intentionally toggling
        # controllers at a very high rate; the latest state is retried next pass.
        for _attempt in range(8):
            with self._state_cv:
                desired_topic = self._desired_topic
                if desired_topic is None or desired_topic == self._applied_topic:
                    return
                if (
                    self._set_topic_service_identity_status != _IDENTITY_FOUND
                    or self._set_topic_service_identity is None
                ):
                    return
                service_generation = self._set_topic_service_generation
                service_uri = self._set_topic_service_identity[0]

            request = self._set_joy_topic.request_class()
            request.topic_name = desired_topic
            try:
                response = self._call_service_bounded(
                    self._set_joy_topic,
                    service_uri,
                    request,
                )
            except Exception as exception:
                rospy.logwarn_throttle(
                    5.0,
                    "Set Joy topic service unavailable: %s" % exception,
                )
                return

            if not response.success:
                rospy.logwarn_throttle(
                    5.0,
                    "Set Joy topic rejected: %s" % response.message,
                )
                return

            with self._state_cv:
                # Never let a late reply from an old service provider mark the
                # new provider's real subscription as already reconciled.
                if (
                    self._set_topic_service_generation != service_generation
                    or self._set_topic_service_identity_status != _IDENTITY_FOUND
                ):
                    continue
                self._applied_topic = desired_topic
                latest_desired_topic = self._desired_topic

            rospy.loginfo("Joy topic switched to %s", desired_topic)
            if latest_desired_topic == desired_topic:
                return

        rospy.logwarn(
            "Joy topic kept changing while applying; retry latest state next pass"
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

    def _refresh_service_registrations(self):
        set_topic_probe = self._lookup_service_identity(
            self._set_joy_topic_service
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
        controller_list_changed = self._commit_identity_probe(
            "controller_list",
            *controller_list_probe,
            controller_probe_event_revision,
        )

        if set_topic_changed:
            rospy.logwarn(
                "Set Joy topic service registration changed; reapplying desired topic"
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
            + 2.0 * self._service_call_timeout_s
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
