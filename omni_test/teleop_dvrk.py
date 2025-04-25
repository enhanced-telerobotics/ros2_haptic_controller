import numpy as np
import dvrk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
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
        self.inv3_btn_subscriber = self.create_subscription(
            Joy, 'delayed_button', self.btn_callback, 10)
        self.jaw_btn_subscriber = self.create_subscription(
            Joy, 'grasp_cmd', self.jaw_btn_callback, 10)
        self.force_lock_publisher = self.create_publisher(
            Bool, '/inv3/HD_force_lock', 10)
        self.target_pose = None  # Stores the latest pose message
        self.initial_pose = None  # Stores the last received pose message
        self.jaw_closed = False
        self.study_started = False
        self.is_clutch = False
        self.diff = np.zeros(3)  # Difference between target and current position
        self.teleop_runner = None  # Placeholder for teleoperation runner
        self.lock = threading.Lock()  # Prevents race conditions
        self.clutch_safety = False 
        self.goal = None
        self.current_cp = None
        self.R_to_HSRV = PyKDL.Rotation(
            PyKDL.Vector(-1.0, 0.0, 0.0),
            PyKDL.Vector(0.0, 0.866025404, 0.5),
            PyKDL.Vector(0.0, 0.5, -0.866025404)
        )
        self.thread_running = True
        threading.Thread(target=self.update_dvrk_cp, daemon=True).start()
        
    def btn_callback(self, msg):
        with self.lock:
            if msg.buttons[1] == 1:
                self.study_started = not self.study_started
            if not self.clutch_safety:
                self.is_clutch = bool(msg.buttons[0]) # press to toggle clutch mode
    def jaw_btn_callback(self, msg):
        with self.lock:
            self.jaw_closed = bool(msg.buttons[6])
    def robot_callback(self, msg):
        with self.lock:
            # Update initial_pose to the last target_pose
            self.initial_pose = self.target_pose
            # Update target_pose to the latest pose message
            self.target_pose = msg  
            # Access the current Cartesian position of the arm
            if self.teleop_runner is not None and self.initial_pose is not None and self.target_pose is not None:

                self.diff = np.array([(self.target_pose.position.y - self.initial_pose.position.y),
                            -(self.target_pose.position.x - self.initial_pose.position.x),
                            (self.target_pose.position.z - self.initial_pose.position.z)])
                
                if np.linalg.norm(self.diff) > 0.01:
                    self.get_logger().info(f"Data loss detected, resetting goal position. Diff exceed {np.linalg.norm(self.diff) * 1000} mm")
                    return
                # Update the goal position based on clutch mode
                if self.current_cp is not None:
                    if not self.is_clutch:
                        self.current_cp.p += self.R_to_HSRV * PyKDL.Vector(self.diff[0], self.diff[1], self.diff[2])
                        self.teleop_runner.arm.servo_cp(self.current_cp)
                
                # Calculate and log FPS
    def update_dvrk_cp(self):
        while rclpy.ok() and self.thread_running:
            with self.lock:
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
        self.ecm = dvrk.ecm(ral=ral, arm_name='ECM')
        time.sleep(0.2)
    

    
    def home(self):
        self.ral.check_connections()
        print('> Starting enable')
        if not self.arm.enable(10) or not self.ecm.enable(10):
            print('  ! Failed to enable within 10 seconds')
            self.ral.shutdown()
        print('> Starting home')
        if not self.arm.home(10) or not self.ecm.home(10):
            print('  ! Failed to home within 10 seconds')
            self.ral.shutdown()
        print('< Home complete')


    def teleop_servo_cp(self):
        print('> Waiting for target pose from subscriber...')

        sleep_rate = self.ral.create_rate(1.0 / self.period)
        self.prepare_cartesian()
        while rclpy.ok():
            # Get the latest pose from subscriber safely
            with self.teleop.lock:
                self.userstudy_controller(user_start=self.teleop.study_started)
                if self.teleop.jaw_closed:
                    # self.arm.jaw.servo_jp(np.array([np.radians(0)]))
                    self.arm.jaw.servo_jp(np.array([np.radians(-5)]))
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
        self.safe_force_lock()
        if ((self.arm_name.endswith('PSM1')) or (self.arm_name.endswith('PSM2'))):
            print('  > preparing for cartesian motion')
            # set in position joint mode
            joint_goal[0] = np.radians(55)
            joint_goal[1] = np.radians(0)
            joint_goal[2] = 0.12
            joint_goal[3] = np.radians(0)
            joint_goal[4] = -0.00505554962626077
            joint_goal[5] = -0.9518453492970927
            start_angle = np.radians(20.0)
            self.arm.move_jp(joint_goal).wait()
            self.arm.jaw.open(angle = start_angle).wait()
            print('  < ready for cartesian mode')
            print('  > preparing for camera position')
            # set in position joint mode
            camera_goal[0] = np.radians(0)
            camera_goal[1] = np.radians(-5)
            camera_goal[2] = 0.10
            camera_goal[3] = np.radians(0)
            self.ecm.move_jp(camera_goal).wait()
            print('  > camera position ready')
            print('  < ready for cartesian mode')
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
        self.teleop.clutch_safety = False
        self.teleop.force_lock_publisher.publish(Bool(data=False))

    def safe_force_lock(self):
        self.teleop.clutch_safety = True
        self.teleop.is_clutch = True
        self.teleop.force_lock_publisher.publish(Bool(data=True))
        time.sleep(1)
        print('  > force lock')
    
    def reset_study(self):
        self.home()
        # self.safe_force_lock()
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

