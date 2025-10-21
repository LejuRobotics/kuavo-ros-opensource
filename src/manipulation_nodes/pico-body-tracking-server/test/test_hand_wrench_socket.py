#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Socket test client for hand_wrench testing with SET/GET loop and detailed results

This test script connects to a Pico VR server and sends VRCommand messages
to test hand wrench functionality with continuous SET/GET operations.
"""

import socket
import time
import sys
import os
import threading
import queue
from typing import Optional

# ANSI color codes for terminal output
class Colors:
    GREEN = '\033[92m'
    ENDC = '\033[0m'

# Add the scripts directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts', 'core', 'ros'))

try:
    import body_tracking_extended_pb2 as proto
    import hand_wrench_srv_pb2 as wrench_pb2
    from google.protobuf.timestamp_pb2 import Timestamp
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure you're running this from the project root directory")
    sys.exit(1)


class HandWrenchTestClient:
    """Test client for hand wrench functionality with detailed result printing
    
    This client uses separate threads for sending and receiving:
    - Send thread: Handles message transmission from queue
    - Receive thread: Continuously listens for responses
    - Main thread: Controls test logic and processes responses
    """
    
    def __init__(self, server_ip: str = '127.0.0.1', server_port: int = 12345):
        """
        Initialize the test client
        
        Args:
            server_ip: Server IP address
            server_port: Server port
        """
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        self.message_id = 0
        self.response_queue = queue.Queue()
        self.send_queue = queue.Queue()
        self.receive_thread = None
        self.send_thread = None
        self.running = False
        
    def connect(self) -> bool:
        """Connect to the server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(0.1)  # Short timeout for receive thread
            print(f"Connected to server at {self.server_ip}:{self.server_port}")
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
            self.receive_thread.start()
            self.send_thread.start()
            return True
        except Exception as e:
            print(f"Failed to connect to server: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the server"""
        self.running = False
        # Signal threads to stop
        self.send_queue.put(None)  # Signal send thread to stop
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        if self.socket:
            self.socket.close()
            self.socket = None
        print("Disconnected from server")
    
    def create_hand_wrench_message(self, 
                                 operation: wrench_pb2.ItemMassForceOperation,
                                 item_mass_force: Optional[wrench_pb2.ItemMassForce] = None) -> proto.VRData:
        """
        Create a VRCommand message with hand wrench data
        
        Args:
            operation: The operation type (GET or SET)
            item_mass_force: Optional item mass force data for SET operations
            
        Returns:
            VRData protobuf message
        """
        # Create main message
        vr_data = proto.VRData()
        
        # Create header
        header = proto.Header()
        timestamp = Timestamp()
        timestamp.GetCurrentTime()
        header.timestamp.CopyFrom(timestamp)
        header.id = self.message_id
        vr_data.header.CopyFrom(header)
        
        # Create VR command data
        vr_command = proto.VrCommandData()
        
        # Create hand wrench request
        hand_wrench_request = wrench_pb2.ItemMassForceRequest()
        hand_wrench_request.operation = operation
        
        if item_mass_force:
            hand_wrench_request.data.CopyFrom(item_mass_force)
        
        vr_command.item_mass_force_request.CopyFrom(hand_wrench_request)
        
        vr_data.vr_command.CopyFrom(vr_command)
        
        self.message_id += 1
        return vr_data
    
    def _send_loop(self):
        """Continuous send loop running in separate thread"""
        print("Send thread started")
        while self.running:
            try:
                message = self.send_queue.get(timeout=0.1)
                if message is None:  # Stop signal
                    break
                
                data = message.SerializeToString()
                self.socket.sendto(data, (self.server_ip, self.server_port))
                print(f"[SEND] Sent message ID: {message.header.id}")
                
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    print(f"[SEND] Error sending message: {e}")
        print("Send thread stopped")
    
    def send_message(self, message: proto.VRData) -> bool:
        """
        Queue a message to be sent by the send thread
        
        Args:
            message: VRData protobuf message
            
        Returns:
            True if message was queued successfully
        """
        try:
            self.send_queue.put(message)
            return True
        except Exception as e:
            print(f"Failed to queue message: {e}")
            return False
    
    def _receive_loop(self):
        """Continuous receive loop running in separate thread"""
        print("Receive thread started")
        while self.running:
            try:
                data, addr = self.socket.recvfrom(4096)
                response = proto.VRData()
                response.ParseFromString(data)
                
                # Check if this is a SET/GET response and print immediately
                if response.HasField('robot_data'):
                    robot_data = response.robot_data
                    if robot_data.HasField('item_mass_force_response'):
                        print(f"\n{Colors.GREEN}--- {self._get_operation_type(robot_data.item_mass_force_response.operation)} Response Details ---{Colors.ENDC}")
                        self._current_response = response
                        self.print_response_details()
                
                self.response_queue.put(response)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[RECV] Error receiving response: {e}")
        print("Receive thread stopped")
    
    def wait_for_response(self, timeout: float = 3.0) -> Optional[proto.VRData]:
        """
        Wait for response from the receive thread queue
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            VRData protobuf message or None if timeout/error
        """
        try:
            response = self.response_queue.get(timeout=timeout)
            return response
        except queue.Empty:
            print(f"Timeout waiting for response ({timeout}s)")
            return None
    
    def receive_response(self, timeout: float = 3.0) -> Optional[proto.VRData]:
        """
        Receive response from server (legacy method for backward compatibility)
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            VRData protobuf message or None if timeout/error
        """
        return self.wait_for_response(timeout)
    
    def print_hand_wrench_details(self, hand_wrench: wrench_pb2.ItemMassForce, prefix: str = ""):
        """
        Print detailed hand wrench configuration
        
        Args:
            hand_wrench: ItemMassForce protobuf message
            prefix: Prefix for each line
        """
        print(f"{prefix}Case Name: {hand_wrench.case_name}")
        print(f"{prefix}Description: {hand_wrench.description}")
        print(f"{prefix}Item Mass: {hand_wrench.item_mass} kg")
        print(f"{prefix}Force X: {hand_wrench.lforce_x} N")
        print(f"{prefix}Force Y: {hand_wrench.lforce_y} N")
        print(f"{prefix}Force Z: {hand_wrench.lforce_z} N")
    
    def _get_operation_type(self, operation: wrench_pb2.ItemMassForceOperation) -> str:
        """Get operation type as readable string"""
        if operation == wrench_pb2.ItemMassForceOperation.GET:
            return "GET"
        elif operation == wrench_pb2.ItemMassForceOperation.SET:
            return "SET"
        return "UNKNOWN"
    
    def print_response_details(self):
        """
        Print detailed response information from the current response
        """
        if hasattr(self, '_current_response'):
            response = self._current_response
            if response.HasField('robot_data'):
                robot_data = response.robot_data
                if robot_data.HasField('item_mass_force_response'):
                    hand_wrench_response = robot_data.item_mass_force_response
                    
                    # Get status as string
                    status_str = "UNKNOWN"
                    if hand_wrench_response.status == hand_wrench_response.OperationStatus.SUCCESS:
                        status_str = "SUCCESS"
                    elif hand_wrench_response.status == hand_wrench_response.OperationStatus.ERROR:
                        status_str = "ERROR"
                    elif hand_wrench_response.status == hand_wrench_response.OperationStatus.TIMEOUT:
                        status_str = "TIMEOUT"
                    elif hand_wrench_response.status == hand_wrench_response.OperationStatus.INVALID_REQUEST:
                        status_str = "INVALID_REQUEST"
                    
                    print(f"{Colors.GREEN}Message ID: {response.header.id}{Colors.ENDC}")
                    print(f"{Colors.GREEN}Operation: {self._get_operation_type(hand_wrench_response.operation)}{Colors.ENDC}")
                    print(f"{Colors.GREEN}Status: {status_str}{Colors.ENDC}")
                    print(f"{Colors.GREEN}Description: {hand_wrench_response.description}{Colors.ENDC}")
                    
                    if hand_wrench_response.item_mass_forces:
                        print(f"{Colors.GREEN}Number of configurations: {len(hand_wrench_response.item_mass_forces)}{Colors.ENDC}")
                        for i, config in enumerate(hand_wrench_response.item_mass_forces):
                            print(f"\n{Colors.GREEN}Configuration {i+1}:{Colors.ENDC}")
                            self.print_hand_wrench_details(config, f"  {Colors.GREEN}")
                    else:
                        print(f"{Colors.GREEN}No configurations returned{Colors.ENDC}")
    
    def test_set_and_get_loop(self, iterations: int = 5, delay: float = 2.0):
        """
        Test SET and GET operations in a loop with detailed results
        
        Args:
            iterations: Number of SET/GET cycles
            delay: Delay between operations in seconds
        """
        print(f"\n=== Starting SET/GET Loop Test ({iterations} iterations) ===")
        
        for iteration in range(iterations):
            print(f"\n{'='*60}")
            print(f"Iteration {iteration + 1}/{iterations}")
            print(f"{'='*60}")
            
            # Create unique test data for this iteration
            test_mass = 2.0 + iteration * 0.5
            test_force_x = 5.0 + iteration * 2
            test_force_y = 10.0 + iteration * 3
            test_force_z = 3.0 + iteration
            
            # SET operation
            print(f"\n--- SET Operation ---")
            hand_wrench_set = wrench_pb2.ItemMassForce()
            hand_wrench_set.case_name = f"test_case_{iteration}"
            hand_wrench_set.description = f"Test configuration from iteration {iteration}"
            hand_wrench_set.item_mass = test_mass
            hand_wrench_set.lforce_x = test_force_x
            hand_wrench_set.lforce_y = test_force_y
            hand_wrench_set.lforce_z = test_force_z
            
            print("Sending SET request with following data:")
            self.print_hand_wrench_details(hand_wrench_set, "  ")
            
            set_message = self.create_hand_wrench_message(
                operation=wrench_pb2.ItemMassForceOperation.SET,
                item_mass_force=hand_wrench_set
            )
            
            if self.send_message(set_message):
                print("SET request sent successfully")
                
                # Response will be printed immediately when received in receive thread
            else:
                print("Failed to send SET request")
            
            # Delay before GET operation
            print(f"\nWaiting {delay}s before GET operation...")
            time.sleep(delay)
            
            # GET operation
            print(f"\n--- GET Operation ---")
            print("Sending GET request to retrieve all configurations")
            
            get_message = self.create_hand_wrench_message(
                operation=wrench_pb2.ItemMassForceOperation.GET
            )
            
            if self.send_message(get_message):
                print("GET request sent successfully")
                
                # Response will be printed immediately when received in receive thread
            else:
                print("Failed to send GET request")
            
            # Delay before next iteration
            if iteration < iterations - 1:
                print(f"\nWaiting {delay}s before next iteration...")
                time.sleep(delay)
        
        print(f"\n{'='*60}")
        print(f"SET/GET Loop Test completed ({iterations} iterations)")
        print(f"{'='*60}")
    
    def run_test(self, iterations: int = 5, delay: float = 2.0):
        """Run the hand wrench test"""
        print("Starting Hand Wrench SET/GET Loop Test")
        print("=" * 60)
        
        if not self.connect():
            print("Failed to connect to server")
            return
        
        try:
            self.test_set_and_get_loop(iterations, delay)
            
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        except Exception as e:
            print(f"Test failed with error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.disconnect()


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Test hand wrench SET/GET loop with detailed results')
    parser.add_argument('--ip', default='127.0.0.1', help='Server IP address (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=12345, help='Server port (default: 12345)')
    parser.add_argument('--iterations', type=int, default=5, help='Number of SET/GET iterations (default: 5)')
    parser.add_argument('--delay', type=float, default=2.0, help='Delay between operations in seconds (default: 2.0)')
    
    args = parser.parse_args()
    
    client = HandWrenchTestClient(args.ip, args.port)
    client.run_test(args.iterations, args.delay)


if __name__ == "__main__":
    main()