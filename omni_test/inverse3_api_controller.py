#!/usr/bin/env python

import HaplyHardwareAPI
import os
import time
import numpy as np
import threading
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
# Detect and connect to the Inverse3 device
connected_devices = HaplyHardwareAPI.detect_inverse3s()
connected_handles = HaplyHardwareAPI.detect_handles()

if not connected_devices:
    raise RuntimeError("No Inverse3 devices detected.")
if not connected_handles:
    raise RuntimeError("No Inverse3 handles detected.")

print(f"Connected devices: {connected_devices}")
print(f"Detected handles: {connected_handles}")

# Initialize device and handle
com_stream = HaplyHardwareAPI.SerialStream(connected_devices[0])
connected_handles_stream = HaplyHardwareAPI.SerialStream(connected_handles[0])
inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
verse_grip = HaplyHardwareAPI.Handle(connected_handles_stream)
response_to_wakeup = inverse3.device_wakeup_dict()
print("Connected to device {}".format(response_to_wakeup["device_id"])) 
print("Connected to handle {}".format(verse_grip.GetVersegripStatus()))
# Parameters
loop_time = 0.001  # 1ms
# Force model for cube boundaries
def force_cube(cube_center, cube_size, device_position, stiffness):
    """Calculate forces to simulate a cube boundary."""
    f = [0, 0, 0]
    for i in range(3):
        min_bound = cube_center[i] - cube_size / 2
        max_bound = cube_center[i] + cube_size / 2
        if device_position[i] < min_bound:
            f[i] = stiffness * (min_bound - device_position[i])
        elif device_position[i] > max_bound:
            f[i] = stiffness * (max_bound - device_position[i])
    return f

class Inverse3Controller(Node):
    def __init__(self):
        super().__init__('inverse3_controller')
        self.pose_publisher = self.create_publisher(PoseStamped, 'inv3_pose', 10)
        self.btn_publisher = self.create_publisher(Joy, 'inv3_btn', 10)
        self.loop_time = loop_time
        self.cal_file = os.path.join(
            os.path.dirname(__file__), 'device_cal/inverse3_cali_param.json')
        self.R_to_world = self.load_calibration_matrix()
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def load_calibration_matrix(self):
        try:
            with open(self.cal_file, 'r') as f:
                R_to_world = np.array(json.load(f))
                if R_to_world.shape != (3, 3):
                    raise ValueError("Invalid calibration matrix shape.")
        except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
            self.get_logger().warn(f"Calibration file error: {e}. Using identity matrix.")
            R_to_world = np.eye(3)
        self.get_logger().info(f"R_to_world: {R_to_world}")
        return R_to_world

    def control_loop(self):
        """Main control loop for the device."""
        try:
            forces = [0, 0, 0]
            start_time = time.perf_counter()

            while rclpy.ok():
                # Get position and velocity from the device
                position, velocity = inverse3.end_effector_force(forces)
                calibrated_position = position @ self.R_to_world.T
                # Publish the position
                msg = PoseStamped()
                btn = Joy()
                btn.buttons = [0, 0]  # Initialize buttons with a list of zeros
                if verse_grip.GetVersegripStatus()['buttons'] == 1: 
                    btn.buttons[0] = 1
                elif verse_grip.GetVersegripStatus()['buttons'] == 2:
                    btn.buttons[1] = 1
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "inv3_frame"
                # msg.pose.position.x = calibrated_position[0] # Uncomment for dvrk
                # msg.pose.position.y = calibrated_position[1]
                msg.pose.position.x = -calibrated_position[0]
                msg.pose.position.y = -calibrated_position[1]
                msg.pose.position.z = calibrated_position[2]
                self.pose_publisher.publish(msg)
                self.btn_publisher.publish(btn)
                # Compute forces based 
                # on the cube boundary model
                # forces = force_cube([0, -0.14, 0.2], 0.04, calibrated_position, stiffness=1200)

                # Loop timing control
                elapsed_time = time.perf_counter() - start_time
                if elapsed_time < self.loop_time:
                    time.sleep(self.loop_time - elapsed_time)
                start_time = time.perf_counter()

        except KeyboardInterrupt:
            self.get_logger().info("Shutting down gracefully.")
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")

def main():
    """Main entry point of the script."""
    rclpy.init()
    node = Inverse3Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exiting program.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()