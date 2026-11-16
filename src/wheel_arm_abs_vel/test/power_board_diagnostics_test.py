"""Sampling/estop diagnostics, with no serial device or ROS master."""
import importlib.util
from pathlib import Path
from unittest.mock import patch, MagicMock
import unittest
import rospy
from kuavo_msgs.msg import PowerBoardStatus
P=Path(__file__).resolve().parents[1]
def load(name, path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module
query=load('private_battery_query',P/'scripts/power_board/controller/hardware/battery_query.py')
led=load('private_led',P/'scripts/led_strip_service.py')
estop=load('private_estop',P/'scripts/physical_estop_bridge_node.py')
class Diagnostics(unittest.TestCase):
    def test_cached_sample_keeps_original_timestamp(self):
        with patch.object(query.time,'time',return_value=42.):
            status=query._parse_system_status_response(bytes([255,255,1,8,0,0,0,25,24,24,24,0]))
        self.assertEqual(status['_sample_time'],42.)
        for _ in range(3):
            msg=led.LEDStripServiceNode._build_power_board_status_msg(status)
            self.assertEqual(msg.timestamp.to_sec(),42.)
    def test_original_estop_gate_and_stale_exit_snapshot(self):
        pub=MagicMock()
        with patch.object(rospy,'init_node'),patch.object(rospy,'Publisher',return_value=pub),patch.object(rospy,'Subscriber'),patch.object(rospy,'on_shutdown'),patch.object(rospy,'loginfo'),patch.object(rospy,'logerr'),patch.object(rospy,'logwarn') as warning:
            bridge=estop.PhysicalEstopBridge()
            msg=PowerBoardStatus();msg.timestamp=rospy.Time.from_sec(42.)
            msg.stop_int=True;bridge._on_status(msg);pub.publish.assert_not_called()
            msg.stop_int=False;bridge._on_status(msg);pub.publish.assert_not_called()
            msg.stop_int=True;bridge._on_status(msg)
            self.assertEqual(pub.publish.call_count,5)
            bridge._on_status(msg);self.assertEqual(pub.publish.call_count,5)
            with patch.object(estop.time,'time',return_value=50.):bridge._print_exit_snapshot()
            self.assertIn('reason=stale',warning.call_args[0][0])
            msg.timestamp=rospy.Time();bridge._on_status(msg)
            bridge._print_exit_snapshot()
            self.assertEqual(warning.call_args[0][2:4],('UNKNOWN','no_data'))
if __name__=='__main__':unittest.main()
