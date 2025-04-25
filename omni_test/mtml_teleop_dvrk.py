import numpy as np
import dvrk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool
import threading
import time
import PyKDL
import argparse
import crtk


class HD(Node):
    def __init__(self):
        super().__init__('HD_teleop_node')
        self.robot_subscriber = self.create_subscription(
            Pose, 'delayed_pose', self.robot_callback, 10)
        self.study_btn_subscriber = self.create_subscription(
            Joy, 'delayed_button', self.btn_callback, 10)
        self.force_lock_publisher = self.create_publisher(
            Bool, '/inv3/HD_force_lock', 10)
        self.footpedal_clutch_subscriber = self.create_subscription(
            Joy, '/footpedals/clutch', self.clutch_cb, 10)
        self.footpedal_coag_subscriber = self.create_subscription(
            Joy, '/footpedals/coag', self.coag_cb, 10)
        self.target_pose = None  # Stores the latest pose message
        self.initial_pose = None  # Stores the last received pose message
        self.study_started = False
        self.diff = np.zeros(3)  # Difference between target and current position
        self.teleop_runner = None  # Placeholder for teleoperation runner
        self.jaw_closed = False
        self.is_clutch = False
        self.is_coag = False 
        self.goal = None
        self.current_cp = None
        self.thread_running = True
        threading.Thread(target=self.update_dvrk_cp, daemon=True).start()
    
    def clutch_cb(self, msg):
        self.is_clutch = bool(msg.buttons[0]) if msg.buttons else False
    
    def coag_cb(self, msg):
        self.is_coag = bool(msg.buttons[0])
    
    def btn_callback(self, msg):
        if msg.buttons[1] == 1:
            self.study_started = not self.study_started

    def study_btn_callback(self, msg):
        self.jaw_closed = bool(msg.buttons[6])

    def robot_callback(self, msg):
        # Update initial_pose to the last target_pose
        self.initial_pose = self.target_pose
        # Update target_pose to the latest pose message
        self.target_pose = msg
        # Access the current Cartesian position of the arm
        if self.teleop_runner is not None and self.initial_pose is not None and self.target_pose is not None:
            if self.current_cp is not None and not self.is_clutch and self.is_coag:
                self.diff = np.array([
                    self.target_pose.position.x - self.initial_pose.position.x,
                    self.target_pose.position.y - self.initial_pose.position.y,
                    self.target_pose.position.z - self.initial_pose.position.z
                ])
                self.current_cp.p += PyKDL.Vector(*self.diff)
                self.teleop_runner.arm.servo_cp(self.current_cp)

    def update_dvrk_cp(self):
        while rclpy.ok() and self.thread_running:
            if self.teleop_runner is not None:
                try:
                    self.current_cp = self.teleop_runner.arm.setpoint_cp()
                except Exception:
                    self.get_logger().warn('setpoint_cp not ready')
            time.sleep(0.005)

class run_teleoperation:
    def __init__(self, ral, arm_name, teleop, period=0.05):
        print(f'> Configuring dvrk_arm_test for {arm_name}')
        self.ral = ral
        self.arm_name = arm_name
        self.teleop = teleop
        self.period = period
        self.arm = dvrk.psm(ral=ral, arm_name=arm_name)
        self.mtml = dvrk.mtm(ral=ral, arm_name='MTML')
        self.ecm = dvrk.ecm(ral=ral, arm_name='ECM')
        time.sleep(0.2)
    
    def home(self):
        self.ral.check_connections()
        print('> Starting enable')
        if not self.arm.enable(10) or not self.ecm.enable(10) or not self.mtml.enable(10):
            print('  ! Failed to enable within 10 seconds')
            self.ral.shutdown()
        print('> Starting home')
        if not self.arm.home(10) or not self.ecm.home(10) or not self.mtml.home(10):
            print('  ! Failed to home within 10 seconds')
            self.ral.shutdown()
        print('< Home complete')

    def teleop_servo_cp(self):
        print('> Waiting for target pose from subscriber...')

        sleep_rate = self.ral.create_rate(1.0 / self.period)
        self.prepare_cartesian()
        while rclpy.ok():
            # Get the latest pose from subscriber safely
            self.userstudy_controller(user_start=self.teleop.study_started)
            try:
                if self.teleop.is_coag:
                    self.mtml.body.servo_cf(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
                    mtml_cp = self.mtml.setpoint_cp()
                else:
                    psm_cp = self.arm.setpoint_cp()
                    goal = PyKDL.Frame(psm_cp.M, mtml_cp.p)
                    # self.mtml.lock_orientation_as_is()
                    self.mtml.move_cp(goal)
                self.jaw_control()
            except Exception:
                continue
            # Ensure loop runs at desired frequency
            sleep_rate.sleep()

    def jaw_control(self):
        jaw_rad = self.mtml.gripper.measured_js()[0]
        # Clamp between -20° and 80°
        jaw_deg = np.clip(np.degrees(jaw_rad), -20.0, 80.0)
        target_jaw = np.radians(jaw_deg)
        # Smoothly step toward target
        step = 10.0 * self.period  # Adjust speed; ~2 rad/sec
        if self.arm.jaw.measured_js()[0] < target_jaw:
            new_jaw = min(self.arm.jaw.measured_js()[0] + step, target_jaw)
        else:
            new_jaw = max(self.arm.jaw.measured_js()[0] - step, target_jaw)
        self.arm.jaw.servo_jp(new_jaw)

    def prepare_cartesian(self):
        # Prepare arm for Cartesian motion
        self.safe_force_lock()
        if self.arm_name.endswith(('PSM1', 'PSM2')):
            print('  > preparing for cartesian motion')
            self.arm.move_jp(np.array([np.radians(55), 0, 0.12, 0, -0.005, -0.95])).wait()
            self.arm.jaw.open(angle=np.radians(20)).wait()
            print('  < ready for cartesian mode')

            print('  > preparing for camera position')
            self.ecm.move_jp(np.array([0, np.radians(-5), 0.10, 0])).wait()
            print('  < camera position ready')
        time.sleep(0.5)

    def userstudy_controller(self, user_start=False):
        """
        This function is used to servo the arm to a specific Cartesian position.
        It toggles on and off based on user study flow.
        """
        if user_start:
            if not hasattr(self, '_study_started') or not self._study_started:
                self.release_force_lock()
                time.sleep(0.5)
                self._study_started = True
                print('  > single trial started')
        else:
            if hasattr(self, '_study_started') and self._study_started:
                self.reset_study()
                self._study_started = False
                print('  > single trial ended')

    def release_force_lock(self):
        self.teleop.is_coag = True
        self.teleop.force_lock_publisher.publish(Bool(data=False))

    def safe_force_lock(self):
        self.teleop.is_coag = False
        self.teleop.is_clutch = True
        self.teleop.force_lock_publisher.publish(Bool(data=True))
        time.sleep(1)
        print('  > force lock')
    
    def reset_study(self):
        self.home()
        self.prepare_cartesian()

    def run(self):
        self.home()
        self.teleop_servo_cp()

    def on_shutdown(self):
        print('>> User-defined shutdown callback')
        self.ral.shutdown()


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
    HD_node = HD()
    HD_thread = threading.Thread(target=rclpy.spin, args=(HD_node,))
    HD_thread.daemon = True  # Ensures it exits with the main script
    HD_thread.start()

    # Initialize CRTK
    ral = crtk.ral('dvrk_teleop')
    application = run_teleoperation(
        ral, args.arm if args.arm else 'PSM1', HD_node, args.period)
    HD_node.teleop_runner = application
    ral.on_shutdown(application.on_shutdown)
    ral.spin_and_execute(application.run)

    # Shutdown ROS 2 properly
    rclpy.shutdown()
    HD_node.thread_running = False  # Stop the update thread
    HD_thread.join()
