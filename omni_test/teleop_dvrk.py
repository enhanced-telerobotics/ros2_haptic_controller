import numpy as np
import dvrk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
import threading
import time
import PyKDL
import argparse
import crtk



class Omni(Node):
    def __init__(self):
        super().__init__('omni_teleop_node')
        self.robot_subscriber = self.create_subscription(
            Pose, 'delayed_pose', self.robot_callback, 10)
        self.jaw_subscriber = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.target_pose = None  # Stores the latest pose message
        self.initial_pose = None  # Stores the last received pose message
        self.jaw_closed = False
        self.lock = threading.Lock()  # Prevents race conditions
    def joy_callback(self, msg):
        with self.lock:
            # Check if the jaw button is pressed
            if msg.buttons[0] == 1:
                self.jaw_closed = True
            else:
                self.jaw_closed = False
    def robot_callback(self, msg):
        with self.lock:
            # Update initial_pose to the last target_pose
            self.initial_pose = self.target_pose
            self.target_pose = msg  # Update target_pose to the latest pose message
class run_teleoperation:
    def __init__(self, ral, arm_name, teleop, period=0.0025):
        print(f'> Configuring dvrk_arm_test for {arm_name}')
        self.ral = ral
        self.arm_name = arm_name
        self.teleop = teleop
        self.period = period
        self.arm = dvrk.psm(ral=ral, arm_name=arm_name)
        self.ecm = dvrk.ecm(ral=ral, arm_name='ECM')
        time.sleep(0.2)

    def home(self):
        self.ral.check_connections()
        print('> Starting enable')
        if not self.arm.enable(10):
            print('  ! Failed to enable within 10 seconds')
            self.ral.shutdown()
        print('> Starting home')
        if not self.arm.home(10):
            print('  ! Failed to home within 10 seconds')
            self.ral.shutdown()
        print('< Home complete')
        # utility to position tool/camera deep enough before cartesian examples

    def teleop_servo_cp(self):
        print('> Waiting for target pose from subscriber...')

        sleep_rate = self.ral.create_rate(1.0 / self.period)
        self.prepare_cartesian()
        while rclpy.ok():
            # Get the latest pose from subscriber safely
            with self.teleop.lock:
                target_pose = self.teleop.target_pose
                initial_pose = self.teleop.initial_pose
                jaw_closed = self.teleop.jaw_closed
            if target_pose is not None:
                # Create a new goal from received pose
                goal = PyKDL.Frame()
                omni_translation = PyKDL.Frame()
                cp = self.arm.setpoint_cp()
                if initial_pose is not None:
                    omni_translation.p = PyKDL.Vector((target_pose.position.x - initial_pose.position.x),
                                                      target_pose.position.y - initial_pose.position.y,
                                                      target_pose.position.z - initial_pose.position.z)
                    # initial_rotation = PyKDL.Rotation.RotZ(math.radians(180)) * PyKDL.Rotation.Quaternion(
                    #     initial_pose.orientation.x,
                    #     initial_pose.orientation.y,
                    #     initial_pose.orientation.z,
                    #     initial_pose.orientation.w)
                    # target_rotation = PyKDL.Rotation.Quaternion(
                    #     target_pose.orientation.x,
                    #     target_pose.orientation.y,
                    #     target_pose.orientation.z,
                    #     target_pose.orientation.w)
                    # omni_flip = PyKDL.Rotation(1, 0, 0,
                    #                    0, 1, 0,
                    #                    0, 0, 1)
                    
                    # # target_rotation = omni_flip * target_rotation * omni_flip
                    # initial_rotation = omni_flip * initial_rotation * omni_flip
                    # omni_translation.M  = target_rotation
                else:
                    omni_translation.p = PyKDL.Vector(0.0, 0.0, 0.0)
                    omni_translation.M = cp.M
                
                # goal.M = PyKDL.Rotation.Quaternion(target_pose.orientation.x,
                #                                    target_pose.orientation.y,
                #                                    target_pose.orientation.z,
                #                                    target_pose.orientation.w)
                dvrk_rotation = PyKDL.Rotation.Quaternion(
                    np.sqrt(0.5),
                    0,
                    np.sqrt(0.5),
                    0)
                goal.p = cp.p + omni_translation.p
                goal.M = cp.M
                self.arm.servo_cp(goal)
                if jaw_closed:
                    self.arm.jaw.servo_jp(np.array([np.radians(0)]))
                else:
                    self.arm.jaw.servo_jp(np.array([np.radians(30)]))
                # Ensure loop runs at desired frequency
                sleep_rate.sleep()

    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        jp = self.arm.setpoint_jp()
        ecm_jp = self.ecm.setpoint_jp()

        joint_goal = np.copy(jp)
        camera_goal = np.copy(ecm_jp)
        if ((self.arm_name.endswith('PSM1')) or (self.arm_name.endswith('PSM2'))):
            print('  > preparing for cartesian motion')
            # set in position joint mode
            joint_goal[0] = np.radians(55)
            joint_goal[1] = np.radians(0)
            joint_goal[2] = 0.12
            joint_goal[3] = np.radians(0)
            joint_goal[4] = -0.00505554962626077
            joint_goal[5] = -0.9518453492970927
            start_angle = np.radians(30.0)
            self.arm.move_jp(joint_goal).wait()
            self.arm.jaw.open(angle = start_angle).wait()
            print('  < ready for cartesian mode')
            print('  > preparing for camera position')
            # set in position joint mode
            camera_goal[0] = np.radians(0)
            camera_goal[1] = np.radians(-5)
            camera_goal[2] = 0.20
            camera_goal[3] = np.radians(0)
            self.ecm.move_jp(camera_goal).wait()
            print('  > camera position ready')
            print('  < ready for cartesian mode')
            time.sleep(0.5)

    def run(self):
        self.home()
        self.teleop_servo_cp()

    def on_shutdown(self):
        print('>> User-defined shutdown callback')


if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str,
                        choices=['ECM', 'MTML', 'MTMR',
                                 'PSM1', 'PSM2', 'PSM3'],
                        default='PSM1',
                        help='Arm name corresponding to ROS topics')
    parser.add_argument('-p', '--period', type=float, default=0.01,
                        help='Period used for loops using servo commands')
    args, unknown = parser.parse_known_args()

    # Initialize ROS 2
    rclpy.init(args=unknown)

    # Start Teleop Node in a Separate Thread
    omni_node = Omni()
    omni_thread = threading.Thread(target=rclpy.spin, args=(omni_node,))
    omni_thread.daemon = True  # Ensures it exits with the main script
    omni_thread.start()

    # Initialize CRTK
    ral = crtk.ral('dvrk_arm_test')
    application = run_teleoperation(
        ral, args.arm if args.arm else 'PSM1', omni_node, args.period)
    ral.on_shutdown(application.on_shutdown)
    ral.spin_and_execute(application.run)

    # Shutdown ROS 2 properly
    rclpy.shutdown()
    omni_thread.join()

