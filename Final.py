'''
Created  July 19, 2024
Air Force Research Lab CAMS Labratory
@author: Sagar Shah and Dr. Mike Chapman
'''

# Copyright (c) Air Force Research Lab 2024.  All rights reserved.

import argparse
import sys
import os
import time

from RibbonCutwoKnifeFinal import *
from Autowalk import *

import cv2
import numpy as np

import google.protobuf.wrappers_pb2

import bosdyn.client
from bosdyn.client import math_helpers
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.util
import bosdyn.geometry
import bosdyn.mission.client
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock, get_dock_id
import bosdyn.api.mission
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                         get_se2_a_tform_b)
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.license import LicenseClient


def relative_move(dx, frame_name, robot_command_client, robot_state_client, dy=0.0, dyaw=0.0, stairs=False):
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
    # We do not want to command this goal in body frame because the body will move, thus shifting
    # our goal. Instead, we transform this offset to get the goal position in the output frame
    # (which will be either odom or vision).
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    # Command the robot to go to the goal point in the specified frame. The command will stop at the
    # new position.
    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
        frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
    end_time = 10.0
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=time.time() + end_time)
    # Wait until the robot has reached the goal.
    while True:
        feedback = robot_command_client.robot_command_feedback(cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print('Failed to reach the goal')
            return False
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            print('Arrived at the goal.')
            return True
        time.sleep(1)

    return True

    

def main(argv):
    body_lease = None
    # Configure logging
    bosdyn.client.util.setup_logging()

    # Parse command-line arguments
    parser = argparse.ArgumentParser()

    bosdyn.client.util.add_base_arguments(parser)
    #Autowalk Arguments
    parser.add_argument('--upload_timeout', type=float, default=300.0, dest='upload_timeout',
                        help='Mission upload timeout.')
    parser.add_argument('--mission_timeout', type=float, default=3.0, dest='mission_timeout',
                        help='Mission client timeout.')
    parser.add_argument('--noloc', action='store_true', default=False, dest='noloc',
                        help='Skip initial localization')
    parser.add_argument('--disable_alternate_route_finding', action='store_true', default=False,
                        dest='disable_alternate_route_finding',
                        help='Disable creating alternate-route-finding graph structure')
    parser.add_argument('--disable_directed_exploration', action='store_true', default=False,
                        dest='disable_directed_exploration',
                        help='Disable directed exploration for skipped blocked branches')
    parser.add_argument('--strict_mode', action='store_true', default=False, dest='strict_mode',
                        help='Set strict path following mode')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--time', type=float, default=0.0, dest='duration',
                       help='Time to repeat mission (sec)')
    group.add_argument('--static', action='store_true', default=False, dest='static_mode',
                       help='Stand, but do not run robot')
    parser.add_argument('--walk_directory', dest='walk_directory', required=True,
                                 help='Directory containing graph_nav map and autowalk missions')
    parser.add_argument(
        '--walk_filename', dest='walk_filename', required=True, help=
        'Autowalk mission filename. Script assumes the path to this file is [walk_directory]/missions/[walk_filename]'
    )

    #Ribbon cutting Arguments
    parser.add_argument('--auto-rotate', help='rotate right and front images to be upright',
                        action='store_true')
    parser.add_argument('--image-sources',
                        help='Get image from source(s)', action='append')
    parser.add_argument(
        '--pixel-format', choices=pixel_format_type_strings(),
        help='Requested pixel format of image. If supplied, will be used for all sources.')
    parser.add_argument('-t', '--force-top-down-grasp',
                        help='Force the robot to use a top-down grasp (vector_alignment demo)',
                        action='store_true')
    parser.add_argument('-f', '--force-horizontal-grasp',
                        help='Force the robot to use a horizontal grasp (vector_alignment demo)',
                        action='store_true')
    parser.add_argument('-r', '--force-45-angle-grasp',
                        help='Force the robot to use a 45 degree angled down grasp (rotation_with_tolerance demo)',
                        action='store_true')
    parser.add_argument('-s', '--force-squeeze-grasp',
                        help='Force the robot to use a squeeze grasp', action='store_true')
   
    options = parser.parse_args()

    ################### RUNNING ########################

    # Initialize robot object
    robot = init_robot(options.hostname)
    # Acquire robot lease
    robot.logger.info('Acquiring lease...')
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Initialize power client
        robot.logger.info('Starting power client...')
        power_client = robot.ensure_client(PowerClient.default_service_name)
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

        # Initialize clients

        robot.logger.info('Powering on robot... This may take a several seconds.')
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Robot power on failed.'
        robot.logger.info('Robot powered on.')

        # Turn on power
        power_on_motors(power_client)


        license_client = robot.ensure_client(LicenseClient.default_service_name)
        if not license_client.get_feature_enabled([DockingClient.default_service_name
                                          ])[DockingClient.default_service_name]:
            robot.logger.error('This robot is not licensed for docking.')
            sys.exit(1)
        options.undock = 'True'
    # Run Autowalk and Ribbon Cutting file

        try:
            dock_id = get_dock_id(robot)
            if dock_id is None:
                print('Robot does not seem to be docked')
                # Stand up and wait for the perception system to stabilize
                robot.logger.info('Commanding robot to stand...')
                blocking_stand(command_client, timeout_sec=20)
                countdown(2)
                robot.logger.info('Robot standing.')   
            else:
                print(f'Docked at {dock_id}')
                blocking_undock(robot)
                print('Undocking Success')

            
            main_auto(parser, options, robot, robot_state_client, body_lease, command_client, lease_client)
            print("Finished autowalk")
            #walk_back(robot, distance, command_client)
            #move_backward(command_client)
            dx = -0.7
            relative_move(dx, ODOM_FRAME_NAME, command_client, robot_state_client)
            
            print("Finished cutting")
    
            #### Power off Motors
    
            robot.logger.info('Sitting down and turning off.')
    
            # Power the robot off. By specifying "cut_immediately=False", a safe power off command
            # is issued to the robot. This will attempt to sit the robot before powering off.
            robot.power_off(cut_immediately=False, timeout_sec=20)
            assert not robot.is_powered_on(), 'Robot power off failed.'
            robot.logger.info('Robot safely powered off.')
    
        except Exception as exc:
            print('Autowalk or ribbon cut failed')
            logger = bosdyn.client.util.get_logger()
            logger.exception('Threw an exception')
            return False



if __name__ == '__main__':

    sys.argv = ['Final.py', '--image-sources', 'hand_color_image', '--walk_directory','C:\\Users\\chapmanm\\Downloads\\Ribbon New.walk','--walk_filename','Ribbon New.walk',
            '--pixel-format', 'PIXEL_FORMAT_RGB_U8','--force-45-angle-grasp','-r', '192.168.80.3']

    print(sys.argv)
    if not main(sys.argv[1:]):
        sys.exit(1)
