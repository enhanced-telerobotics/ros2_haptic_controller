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

# Callback to gather the device state and publish it
@hd_callback
def device_callback():
    global device_state
    position = hd.get_position()
    velocity = hd.get_velocity()
    device_state.position = np.array(position)/1000
    device_state.velocity = velocity
    device_state.button = hd.get_buttons()
    # Create and publish position message as PoseStamped
    pos_msg = PoseStamped()
    pos_msg.header = Header()
    pos_msg.header.stamp = omni_encoder_node.get_clock().now().to_msg()
    pos_msg.pose.position.y, pos_msg.pose.position.z, pos_msg.pose.position.x = -device_state.position[0], device_state.position[1], -device_state.position[2]
    pos_msg.pose.orientation.x = 0.0
    pos_msg.pose.orientation.y = 0.0
    pos_msg.pose.orientation.z = 0.0
    pos_msg.pose.orientation.w = 1.0
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
        self.device_state = device_state

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
