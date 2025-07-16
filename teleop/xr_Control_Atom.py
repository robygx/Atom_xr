import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
import argparse
# from multiprocessing import shared_memory, Value, Array, Lock
from threading import Lock  # 使用 threading.Lock
import threading
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)
import os 
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm_ik import Atom_23_ArmIK
from sshkeyboard import listen_keyboard, stop_listening


# state transition
start_signal = False
running = True
should_toggle_recording = False
is_recording = False
should_send_commands = False  # 新增状态变量，用来控制是否发送命令

def on_press(key):
    global running, start_signal, should_toggle_recording,should_send_commands
    if key == 'r':
        start_signal = True
        logger_mp.info("Program start signal received.")
    elif key == 'q':
        stop_listening()
        running = False
    elif key == 's':
        should_toggle_recording = True
    elif key == 'a':  # 按下 'a' 键时，设置 should_send_commands 为 True
        should_send_commands = True
        logger_mp.info("Ready to send commands.")
    else:
        logger_mp.info(f"{key} was pressed, but no action is defined for this key.")

listen_keyboard_thread = threading.Thread(target=listen_keyboard, kwargs={"on_press": on_press, "until": None, "sequential": False,}, daemon=True)
listen_keyboard_thread.start()

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.lock = Lock()

        # 发布器
        self.left_arm_pub = self.create_publisher(JointState, '/joint_command_left_arm', 10)
        self.right_arm_pub = self.create_publisher(JointState, '/joint_command_right_arm', 10)

        # 订阅器
        self.left_arm_sub = self.create_subscription(JointState, '/joint_states_left_arm', self.left_arm_callback, 10)
        self.right_arm_sub = self.create_subscription(JointState, '/joint_states_right_arm', self.right_arm_callback, 10)

        # 用于存储接收到的关节状态
        self.left_arm_states = None
        self.right_arm_states = None

    def left_arm_callback(self, msg):
        with self.lock:
            self.left_arm_states = msg

    def right_arm_callback(self, msg):
        with self.lock:
            self.right_arm_states = msg

    def send_commands(self, sol_q, sol_tauff):

        # logger_mp.debug(f"sol_q: {sol_q}, sol_tau: {sol_tauff}")

        # 左臂消息
        left_msg = JointState()
        left_msg.header.stamp = self.get_clock().now().to_msg()
        left_msg.name = ["left_motor0", "left_motor1", "left_motor2", "left_motor3", "left_motor4"]
        sol_q[4] = 0.0  # 将左臂最后一个关节位置设为0
        left_msg.position = sol_q[:5].tolist()   
        left_msg.velocity = [0.0] * 5            # 速度设为0
        sol_tauff[4] = 0.0  # 将左臂最后一个关节力矩设为0
        left_msg.effort = sol_tauff[:5].tolist() 
        self.left_arm_pub.publish(left_msg)

        # 右臂消息
        right_msg = JointState()
        right_msg.header.stamp = self.get_clock().now().to_msg()
        right_msg.name = ["right_motor0", "right_motor1", "right_motor2", "right_motor3", "right_motor4"]
        sol_q[9] = 0.0  # 将右臂最后一个关节位置设为0
        right_msg.position = sol_q[5:].tolist()   
        right_msg.velocity = [0.0] * 5            # 速度设为0
        sol_tauff[9] = 0.0  # 将右臂最后一个关节力矩设为0
        right_msg.effort = sol_tauff[5:].tolist() 
        self.right_arm_pub.publish(right_msg)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'save data\'s frequency')
    parser.add_argument('--xr-mode', type=str, choices=['hand', 'controller'], default='controller', help='Select XR device tracking source')

    args = parser.parse_args()
    logger_mp.info(f"args: {args}")
    
    # 初始化ROS2
    rclpy.init()
    arm_controller = ArmController()

    # television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
    tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.xr_mode == 'hand', return_state_data=True, return_hand_rot_data = False)

    # arm
    arm_ik = Atom_23_ArmIK(Visualization = True)

    try:
        logger_mp.info("Please enter the start signal (enter 'r' to start the subsequent program)")
        while not start_signal:
            time.sleep(0.01)
        # arm_ctrl.speed_gradual_max()
        while running:
            start_time = time.time()
            # get input data
            tele_data = tv_wrapper.get_motion_state_data()
            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_arm_pose, tele_data.right_arm_pose)
            # sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_arm_pose, tele_data.right_arm_pose, current_lr_arm_q, current_lr_arm_dq)

            # 如果按下 'a' 键，就执行 arm_controller.send_commands
            if should_send_commands:
                arm_controller.send_commands(sol_q, sol_tauff)

            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
        
            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("KeyboardInterrupt, exiting program...")
    finally:
        listen_keyboard_thread.join()

        # 关闭ROS2节点
        if 'arm_controller' in locals():
            arm_controller.destroy_node()
            
        rclpy.shutdown()

        logger_mp.info("Finally, exiting program...")
        exit(0)
