import numpy as np
import time
import argparse
import cv2
from multiprocessing import shared_memory, Value, Array, Lock
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
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK , Atom_23_ArmIK
from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller, Gripper_Controller
from teleop.robot_control.robot_hand_inspire import Inspire_Controller
from teleop.image_server.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from sshkeyboard import listen_keyboard, stop_listening


# state transition
start_signal = False
running = True
should_toggle_recording = False
is_recording = False
def on_press(key):
    global running, start_signal, should_toggle_recording
    if key == 'r':
        start_signal = True
        logger_mp.info("Program start signal received.")
    elif key == 'q':
        stop_listening()
        running = False
    elif key == 's':
        should_toggle_recording = True
    else:
        logger_mp.info(f"{key} was pressed, but no action is defined for this key.")
listen_keyboard_thread = threading.Thread(target=listen_keyboard, kwargs={"on_press": on_press, "until": None, "sequential": False,}, daemon=True)
listen_keyboard_thread.start()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_dir', type = str, default = './utils/data', help = 'path to save data')
    parser.add_argument('--frequency', type = float, default = 90.0, help = 'save data\'s frequency')

    parser.add_argument('--xr-mode', type=str, choices=['hand', 'controller'], default='controller', help='Select XR device tracking source')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'Atom' ], default='Atom', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex3', 'gripper', 'inspire1'], help='Select end effector controller')

    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording')
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')

    args = parser.parse_args()
    logger_mp.info(f"args: {args}")

    # television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
    tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.xr_mode == 'hand', 
                                 return_state_data=True, return_hand_rot_data = False)

    # arm
    if args.arm == 'Atom':
        arm_ik = Atom_23_ArmIK(Visualization = True)
    elif args.arm == 'G1_29':
        arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        arm_ik = G1_29_ArmIK()
    elif args.arm == 'G1_23':
        arm_ctrl = G1_23_ArmController(simulation_mode=args.sim)
        arm_ik = G1_23_ArmIK()
    elif args.arm == 'H1_2':
        arm_ctrl = H1_2_ArmController(simulation_mode=args.sim)
        arm_ik = H1_2_ArmIK()
    elif args.arm == 'H1':
        arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        arm_ik = H1_ArmIK()

        
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
        logger_mp.info("Finally, exiting program...")
        exit(0)
