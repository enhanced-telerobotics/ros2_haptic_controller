"""
@file inverse3_controller.py
@brief Inverse3 controller for ROS2  
@demo https://demo.haply.co/
@script author: Sai Jiang
@date: 2025-03-29
@device: Inverse3, invoke with:
    systemctl start haply-inverse-service.service
    systemctl enable haply-inverse-service.service
"""

import asyncio
import websockets
import orjson
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
import requests

class Inverse3Node(Node):
    def __init__(self):
        super().__init__('inverse3_node')
        self.position_publisher = self.create_publisher(PoseStamped, 'inv3_pose', 10)
        self.velocity_publisher = self.create_publisher(Vector3, 'inv3_velocity', 10)
        self.button_publisher = self.create_publisher(Joy, 'joy', 10)
        self.last_quaternion = None  # For quaternion smoothing

    def publish_all(self, position, velocity, orientation, buttons):
        # Position message
        pos_msg = PoseStamped()
        pos_msg.header = Header()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = "inv3_frame"
        pos_msg.pose.position.x = float(position.get("x", 0.0))
        pos_msg.pose.position.y = float(position.get("y", 0.0))
        pos_msg.pose.position.z = float(position.get("z", 0.0))

        # Orientation quaternion smoothing
        quat = [
            float(orientation.get("x", 0.0)),
            float(orientation.get("y", 0.0)),
            float(orientation.get("z", 0.0)),
            float(orientation.get("w", 1.0))
        ]
        alpha = 0.1
        if self.last_quaternion is None:
            self.last_quaternion = quat
        else:
            quat = [alpha * q + (1 - alpha) * lq for q, lq in zip(quat, self.last_quaternion)]
            self.last_quaternion = quat

        pos_msg.pose.orientation.x = quat[0]
        pos_msg.pose.orientation.y = quat[1]
        pos_msg.pose.orientation.z = quat[2]
        pos_msg.pose.orientation.w = quat[3]
        self.position_publisher.publish(pos_msg)

        # Velocity message (fixed casting here)
        vel_msg = Vector3()
        vel_msg.x = float(velocity.get("x", 0.0))
        vel_msg.y = float(velocity.get("y", 0.0))
        vel_msg.z = float(velocity.get("z", 0.0))
        self.velocity_publisher.publish(vel_msg)

        # Button message
        button_msg = Joy()
        button_msg.header = Header()
        button_msg.header.stamp = self.get_clock().now().to_msg()
        button_msg.buttons = [int(b) for b in buttons.values()] if buttons else []
        self.button_publisher.publish(button_msg)

    def enable_gravity_compensation(self, device_id, enable=True, gravity_scaling_factor=1.0):
        url = "http://localhost:10000/gravity_compensation"
        payload = {
            "device_id": device_id,
            "enable": enable,
            "gravity_scaling_factor": gravity_scaling_factor
        }
        try:
            response = requests.post(url, json=payload)
            if response.status_code == 200:
                self.get_logger().info(f"Gravity compensation response: {response.json()}")
            else:
                self.get_logger().error(f"Failed to enable gravity compensation: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error during gravity compensation request: {e}")
async def websocket_loop(node: Inverse3Node):
    uri = 'ws://localhost:10001'
    first_message = True
    inverse3_device_id = None
    force = {"x": 0, "y": 0, "z": 0}

    async with websockets.connect(uri) as ws:
        while rclpy.ok():
            response = await ws.recv()
            data = orjson.loads(response)

            inverse3_devices = data.get("inverse3", [])
            verse_grip_devices = data.get("wireless_verse_grip", [])
            inverse3_data = inverse3_devices[0] if inverse3_devices else {}
            verse_grip_data = verse_grip_devices[0] if verse_grip_devices else {}

            if first_message:
                first_message = False
                if not inverse3_data:
                    node.get_logger().error("No Inverse3 device found.")
                    break
                inverse3_device_id = inverse3_data.get("device_id")
                node.get_logger().info(f"Inverse3 device ID: {inverse3_device_id}")
                # Enable gravity compensation on the first message
                node.enable_gravity_compensation(inverse3_device_id, enable=True)

            position = inverse3_data["state"].get("cursor_position", {})
            velocity = inverse3_data["state"].get("cursor_velocity", {})
            buttons = verse_grip_data.get("state", {}).get("buttons", {})
            orientation = verse_grip_data.get("state", {}).get("orientation", {})

            node.publish_all(position, velocity, orientation, buttons)
    
            request_msg = {
                "inverse3": [
                    {
                        "device_id": inverse3_device_id,
                        "commands": {
                            "set_cursor_force": {
                                "values": force
                            }
                        }
                    }
                ]
            }
            await ws.send(orjson.dumps(request_msg))


def main():
    rclpy.init()
    node = Inverse3Node()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, executor.spin)
    try:
        loop.run_until_complete(websocket_loop(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
