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
from scipy.spatial.transform import Rotation as R

# Define the transformation matrix according to DH parameters
def dh_transform(theta, d, a, alpha):
    """
    Calculate the transformation matrix from DH parameters.

    Parameters:
    - theta: rotation angle around the previous z-axis (in radians)
    - d: distance along the previous z-axis
    - a: length of the common normal (distance along the previous x-axis)
    - alpha: angle about the common normal, from the previous z-axis to the new z-axis (in radians)

    Returns:
    - 4x4 NumPy array representing the transformation matrix
    """
    # Define the transformation matrix according to DH parameters
    transform_matrix = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),
         np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -
         np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(
            alpha),                 d],
        [0,              0,
            0,                             1]
    ])

    return transform_matrix


def get_transform(joints, gimbals):
    T = dh_transform(-joints[0], 110, 0, -np.radians(90))
    T = np.dot(T,
               dh_transform(-joints[1], 0, 133.35, 0))
    T = np.dot(T,
               dh_transform(-joints[2]+np.radians(180), 0, 0, np.radians(90)))
    T = np.dot(T,
               dh_transform(-gimbals[0]+np.radians(180), 133.35, 0, np.radians(90)))
    T = np.dot(T,
               dh_transform(-gimbals[1]+np.radians(-90), 0, 0, np.radians(90)))
    T = np.dot(T,
               dh_transform(gimbals[2]+np.radians(90), 0, 0, np.radians(90)))

    return T
# Data class to keep track of the device state and use it in other parts of the code
@dataclass
class DeviceState:
    position: tuple = (0.0, 0.0, 0.0)
    velocity: tuple = (0.0, 0.0, 0.0)
    joints: tuple = (0.0, 0.0, 0.0)
    gimbals: tuple = (0.0, 0.0, 0.0)
    force: tuple = (0.0, 0.0, 0.0)
    transform: list = None
    button: int = 0
    
# Initialize the device state
device_state = DeviceState()
Kp = 0
# Callback to gather the device state and publish it
@hd_callback
def device_callback():
    global device_state
    position = hd.get_position()
    velocity = hd.get_velocity()
    transform = hd.get_transform()
    joints = hd.get_joints()
    gimbals = hd.get_gimbals()
    
    global Kp
    # Convert the transform matrix into a quaternion using scipy


    device_state.position = np.array(position) / 1000
    device_state.velocity = velocity
    device_state.button = hd.get_buttons()
    device_state.joints = [joints[0], joints[1], joints[2]]
    device_state.gimbals = [gimbals[0], gimbals[1], gimbals[2]]
    # Create and publish position message as PoseStamped
    pos_msg = PoseStamped()
    pos_msg.header = Header()
    pos_msg.header.stamp = omni_encoder_node.get_clock().now().to_msg()
    transform = get_transform(
        device_state.joints, np.array(device_state.gimbals))
    pos_msg.pose.position.y, pos_msg.pose.position.z, pos_msg.pose.position.x = device_state.position[0], device_state.position[1], device_state.position[2]
    rotation_matrix = np.array(transform).reshape(4, 4)[:3, :3]
    r = R.from_matrix(rotation_matrix)
    quaternion = r.as_quat()  # Returns [x, y, z, w]
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
