import pyOpenHaptics.hd as hd
from pyOpenHaptics.hd_callback import hd_callback
from pyOpenHaptics.hd_device import HapticDevice
from dataclasses import dataclass
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
import time
from scipy.spatial.transform import Rotation as R

# Data class to keep track of the device state and use it in other parts of the code
@dataclass
class DeviceState:
    position: tuple = (0.0, 0.0, 0.0)
    velocity: tuple = (0.0, 0.0, 0.0)
    force: tuple = (0.0, 0.0, 0.0)
    transform: list = None
    button: int = 0

# Initialize the device state
device_state = DeviceState()
Kp = 0
# Callback to gather the device state and publish it
@hd_callback
def device_callback():
    global device_state, Kp
    position = hd.get_position()
    velocity = hd.get_velocity()
    transform = hd.get_transform()
    damping_factor = 0.002  # Damping factor for velocity

    # if not omni_encoder_node.is_start:
    #     if Kp < 0.2:
    #         Kp += 0.00025
    #     force = Kp * (np.array([0.0, 0.0, 0.0]) - np.array(position)) - damping_factor * np.array(velocity)
    #     if np.linalg.norm(np.array([0.0, 0.0, 0.0]) - np.array(position)) < 5.0:
    #         Kp = 0.5
    #     hd.set_force(force)
    # else:
    #     hd.set_force([0.0, 0.0, 0.0])
    #     Kp = 0

    device_state.position = np.array(position) / 1000
    device_state.velocity = velocity
    device_state.button = hd.get_buttons()
    # Create and publish position message as PoseStamped
    pos_msg = PoseStamped()
    pos_msg.header = Header()
    pos_msg.header.stamp = omni_encoder_node.get_clock().now().to_msg()
    
    # This is the transformation to match the omni's coordinate system with the user study sim
    # pos_msg.pose.position.y, pos_msg.pose.position.z, pos_msg.pose.position.x = -device_state.position[0], device_state.position[1], -device_state.position[2]
    # This is the transformation to match the omni's coordinate system with the dvrk
    pos_msg.pose.position.x, pos_msg.pose.position.y, pos_msg.pose.position.z = device_state.position[0], device_state.position[1], device_state.position[2]
    # Extract rotation matrix from the transform
    rotation_matrix = np.array(transform).reshape(4, 4)[:3, :3]
    
    # Convert rotation matrix to quaternion using scipy
    quaternion = R.from_matrix(rotation_matrix).as_quat()

    # Assign quaternion to pose orientation
    # Apply a simple filter to smooth out fluctuations in the quaternion
    alpha = 0.1  # Smoothing factor
    if not hasattr(device_callback, "last_quaternion"):
        device_callback.last_quaternion = quaternion
    else:
        quaternion = alpha * quaternion + (1 - alpha) * device_callback.last_quaternion
        device_callback.last_quaternion = quaternion

    pos_msg.pose.orientation.x = quaternion[0]
    pos_msg.pose.orientation.y = quaternion[1]
    pos_msg.pose.orientation.z = quaternion[2]
    pos_msg.pose.orientation.w = quaternion[3]
    omni_encoder_node.position_publisher.publish(pos_msg)

    # Create and publish velocity message
    vel_msg = Vector3()
    vel_msg.x, vel_msg.y, vel_msg.z = velocity
    omni_encoder_node.velocity_publisher.publish(vel_msg)

    button_msg = Joy()
    button_msg.header = Header()
    button_msg.header.stamp = omni_encoder_node.get_clock().now().to_msg()
    button_msg.buttons = [device_state.button]
    omni_encoder_node.button_publisher.publish(button_msg)

class OmniEncoderNode(Node):
    def __init__(self):
        super().__init__('omni_encoder_node')
        self.position_publisher = self.create_publisher(PoseStamped, 'omni_pose', 10)
        self.velocity_publisher = self.create_publisher(Vector3, 'device_velocity', 10)
        self.button_publisher = self.create_publisher(Joy, 'joy', 10)
        self.robot_subscriber = self.create_subscription(Vector3, 'robot_cmd', self.robot_callback, 10)
        self.device_state = device_state
        self.is_start = False

    def robot_callback(self, msg):
        if msg.y == 1:
            time.sleep(1)  # Wait 0.2s every time is_start is changed to True
            self.is_start = True
        if msg.z == 1:
            self.is_start = False

def main(args=None):
    global omni_encoder_node
    rclpy.init(args=args)
    omni_encoder_node = OmniEncoderNode()

    # Initialize the haptic device and the callback loop
    device = HapticDevice(callback=device_callback, scheduler_type="async")

    try:
        rclpy.spin(omni_encoder_node)
    except KeyboardInterrupt:
        pass

    # Close the device to avoid segmentation faults
    device.close()
    omni_encoder_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
