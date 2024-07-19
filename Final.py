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
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.util
import bosdyn.geometry
import bosdyn.mission.client
import bosdyn.api.mission
import bosdyn.api.power_pb2 as PowerServiceProto
from bosdyn.api import estop_pb2, geometry_pb2, image_pb2, manipulation_api_pb2
from bosdyn.client.estop import EstopClient
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body, math_helpers
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand, block_until_arm_arrives
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.power import PowerClient, power_on_motors, safe_power_off_motors
from bosdyn.api import robot_state_pb2
from bosdyn.api.autowalk import walks_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2
from bosdyn.api.mission import mission_pb2, nodes_pb2



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

        # Initialize clients
        robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        robot.logger.info('Powering on robot... This may take a several seconds.')
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Robot power on failed.'
        robot.logger.info('Robot powered on.')

        # Turn on power
        power_on_motors(power_client)

        # Stand up and wait for the perception system to stabilize
        robot.logger.info('Commanding robot to stand...')
        blocking_stand(command_client, timeout_sec=20)
        countdown(5)
        robot.logger.info('Robot standing.')

    # Run Autowalk and Ribbon Cutting file

    try:
        main_auto(parser, options, robot, lease_client, robot_state_client)
        print("Finished autowalk")
        main_ribbon(options, robot, command_client, robot_state_client)
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

    sys.argv = ['Final.py', '--image-sources', 'hand_color_image', '--walk_directory','C:\\Users\\chapmanm\\Downloads\\Ribbon walk.walk','--walk_filename','Ribbon walk.walk',
            '--pixel-format', 'PIXEL_FORMAT_RGB_U8','--force-45-angle-grasp','-r', '192.168.80.3']

    print(sys.argv)
    if not main(sys.argv[1:]):
        sys.exit(1)



