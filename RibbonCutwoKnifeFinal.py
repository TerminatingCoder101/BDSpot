'''
Created  July 19, 2024
Air Force Research Lab CAMS Labratory
@author: Sagar Shah and Dr. Mike Chapman
'''

# Copyright (c) Air Force Research Lab 2024.  All rights reserved.

import argparse
import sys
import time

import cv2
import numpy as np

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
from google.protobuf import wrappers_pb2
from bosdyn.api import estop_pb2, geometry_pb2, image_pb2, manipulation_api_pb2
from bosdyn.client.estop import EstopClient
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body, math_helpers
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand, block_until_arm_arrives
from bosdyn.client.robot_state import RobotStateClient


def pixel_format_type_strings():
    names = image_pb2.Image.PixelFormat.keys()
    return names[1:]

def pixel_format_string_to_enum(enum_string):
    return dict(image_pb2.Image.PixelFormat.items()).get(enum_string)

def verify_estop(robot):
    """Verify the robot is not estopped"""

    client = robot.ensure_client(EstopClient.default_service_name)
    if client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
        error_message = 'Robot is estopped. Please use an external E-Stop client, such as the' \
                        ' estop SDK example, to configure E-Stop.'
        robot.logger.error(error_message)
        raise Exception(error_message)

def arm_object_grasp(config, robot, command_client, robot_state_client):
    bosdyn.client.util.setup_logging(config.verbose)

    assert robot.has_arm(), 'Robot requires an arm to run this example.'


    image_client = robot.ensure_client(ImageClient.default_service_name)

    manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)
    

    # Take a picture with a camera
    robot.logger.info('Getting an image from: %s', config.image_sources)
    if config.image_sources:
        # Capture and save images to disk
        pixel_format = pixel_format_string_to_enum(config.pixel_format)
        image_request = [
            build_image_request(source, pixel_format=pixel_format)
            for source in config.image_sources
        ]
        image_responses = image_client.get_image(image_request)

        image = image_responses[0]
        
        num_bytes = 1  # Assume a default of 1 byte encodings.
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
            extension = '.png'
        else:
            if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                num_bytes = 3
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                num_bytes = 4
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                num_bytes = 1
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
                num_bytes = 2
            dtype = np.uint8
            extension = '.jpg'

        img = np.frombuffer(image.shot.image.data, dtype=dtype)

        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            try:
                # Attempt to reshape array into an RGB rows X cols shape.
                img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_bytes))
            except ValueError:
                # Unable to reshape the image data, trying a regular decode.
                img = cv2.imdecode(img, -1)
        else:
            img = cv2.imdecode(img, -1)

    x_coord, y_coord = segmentation_processing(img,extension)

    robot.logger.info(
        f'Picking object at image location ({x_coord}, {y_coord})')
    robot.logger.info('Picking object at image location (%s, %s)', x_coord, y_coord)

    pick_vec = geometry_pb2.Vec2(x= x_coord, y= y_coord)

    # # Build the proto
    # grasp = manipulation_api_pb2.PickObjectInImage(
    #     pixel_xy=pick_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
    #     frame_name_image_sensor=image.shot.frame_name_image_sensor,
    #     camera_model=image.source.pinhole)

    # # Optionally add a grasp constraint.  This lets you tell the robot you only want top-down grasps or side-on grasps.
    # add_grasp_constraint(config, grasp, robot_state_client)

    # # Ask the robot to pick up the object
    # grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)

    # # Send the request
    # cmd_response = manipulation_api_client.manipulation_api_command(
    #     manipulation_api_request=grasp_request)

    # # Get feedback from the robot
    # while True:
    #     feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
    #         manipulation_cmd_id=cmd_response.manipulation_cmd_id)

    #     # Send the request
    #     response = manipulation_api_client.manipulation_api_feedback_command(
    #         manipulation_api_feedback_request=feedback_request)

    #     print(
    #         f'Current state: {manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state)}'
    #     )

    #     if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
    #         break

    #     time.sleep(0.25)

    # Optionally populate the offset distance parameter.
    if config.distance is None:
        offset_distance = None
    else:
        offset_distance = wrappers_pb2.FloatValue(value=config.distance)

    # Build the proto
    walk_to = manipulation_api_pb2.WalkToObjectInImage(
        pixel_xy=pick_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
        frame_name_image_sensor=image.shot.frame_name_image_sensor,
        camera_model=image.source.pinhole, offset_distance=offset_distance)

    # Ask the robot to pick up the object
    walk_to_request = manipulation_api_pb2.ManipulationApiRequest(
        walk_to_object_in_image=walk_to)

    # Send the request
    cmd_response = manipulation_api_client.manipulation_api_command(
        manipulation_api_request=walk_to_request)

    # Get feedback from the robot
    while True:
        time.sleep(0.25)
        feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
            manipulation_cmd_id=cmd_response.manipulation_cmd_id)

        # Send the request
        response = manipulation_api_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_request)

        print('Current state: ',
                manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

        if response.current_state == manipulation_api_pb2.MANIP_STATE_DONE:
            break


    robot.logger.info('Finished grasp.')

    stow = RobotCommandBuilder.arm_stow_command()

    # Issue the command via the RobotCommandClient
    stow_command_id = command_client.robot_command(stow)

    robot.logger.info('Stow command issued.')
    block_until_arm_arrives(command_client, stow_command_id, 3.0)

    time.sleep(1.0)


def add_grasp_constraint(config, grasp, robot_state_client):
    # There are 3 types of constraints:
    #   1. Vector alignment
    #   2. Full rotation
    #   3. Squeeze grasp
    #
    # You can specify more than one if you want and they will be OR'ed together.

    # For these options, we'll use a vector alignment constraint.
    use_vector_constraint = config.force_top_down_grasp or config.force_horizontal_grasp

    # Specify the frame we're using.
    grasp.grasp_params.grasp_params_frame_name = VISION_FRAME_NAME

    if use_vector_constraint:
        if config.force_top_down_grasp:
            # Add a constraint that requests that the x-axis of the gripper is pointing in the
            # negative-z direction in the vision frame.

            # The axis on the gripper is the x-axis.
            axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

            # The axis in the vision frame is the negative z-axis
            axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=-1)

        if config.force_horizontal_grasp:
            # Add a constraint that requests that the y-axis of the gripper is pointing in the
            # positive-z direction in the vision frame.  That means that the gripper is constrained to be rolled 90 degrees and pointed at the horizon.

            # The axis on the gripper is the y-axis.
            axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)

            # The axis in the vision frame is the positive z-axis
            axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=1)

        # Add the vector constraint to our proto.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
            axis_on_gripper_ewrt_gripper)
        constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
            axis_to_align_with_ewrt_vo)

        # We'll take anything within about 10 degrees for top-down or horizontal grasps.
        constraint.vector_alignment_with_tolerance.threshold_radians = 0.17

    elif config.force_45_angle_grasp:
        # Demonstration of a RotationWithTolerance constraint.  This constraint allows you to
        # specify a full orientation you want the hand to be in, along with a threshold.
        #
        # You might want this feature when grasping an object with known geometry and you want to
        # make sure you grasp a specific part of it.
        #
        # Here, since we don't have anything in particular we want to grasp,  we'll specify an
        # orientation that will have the hand aligned with robot and rotated down 45 degrees as an
        # example.

        # First, get the robot's position in the world.
        robot_state = robot_state_client.get_robot_state()
        vision_T_body = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)

        # Rotation from the body to our desired grasp.
        body_Q_grasp = math_helpers.Quat.from_pitch(0.785398)  # 45 degrees
        vision_Q_grasp = vision_T_body.rotation * body_Q_grasp

        # Turn into a proto
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.rotation_with_tolerance.rotation_ewrt_frame.CopyFrom(vision_Q_grasp.to_proto())

        # We'll accept anything within +/- 10 degrees
        constraint.rotation_with_tolerance.threshold_radians = 0.17

    elif config.force_squeeze_grasp:
        # Tell the robot to just squeeze on the ground at the given point.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.squeeze_grasp.SetInParent()


def segmentation_processing(img, extension):
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

    mask_red = mask_red1 + mask_red2

    result_image = cv2.bitwise_and(img, img, mask=mask_red)

    img = cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB)
    image = img.copy()

    # Convert mask_red to a format compatible with cvtColor
    # plt.imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
    # plt.show()

    gray_img = cv2.cvtColor(result_image, cv2.COLOR_RGB2GRAY)

    _, black_white = cv2.threshold(gray_img, 1, 255, cv2.THRESH_BINARY)

    blur = cv2.GaussianBlur(black_white, (9,9), 0)

    contours, _ = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables to store the largest horizontal line
    max_width = 0
    best_bbox = None
    cv2.drawContours(image, contours, -1, (255,255, 255), 1)  # Blue color in BGR


    # Iterate through contours to find the longest horizontal line
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w > max_width:
            max_width = w
            best_bbox = (x, y, x + w, y + h)

    # Draw the bounding box on the image
    if best_bbox:
        cv2.rectangle(image, (best_bbox[0], best_bbox[1]), (best_bbox[2], best_bbox[3]), (0, 0, 255), 2)  # Red color in BGR
        center_x = (best_bbox[0] + best_bbox[2]) // 2
        center_y = (best_bbox[1] + best_bbox[3]) // 2

        cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), 2)
        print(f"Coordinates: ({center_x},{center_y})")

    # plt.imshow(image)
    # plt.show()
    # image_saved_path = image.source.name

    # cv2.imwrite('C:\\Users\\chapmanm\\Desktop\\Ribbon_images\\'+ image_saved_path + str(counter) + extension, img)
    # print("Wrote out image to file")
    return center_x, center_y


def main_ribbon(options,robot,command_client, robot_state_client ):
    num = 0
    if options.force_top_down_grasp:
        num += 1
    if options.force_horizontal_grasp:
        num += 1
    if options.force_45_angle_grasp:
        num += 1
    if options.force_squeeze_grasp:
        num += 1

    if num > 1:
        print('Error: cannot force more than one type of grasp.  Choose only one.')
        sys.exit(1)

    try:
        arm_object_grasp(options, robot, command_client, robot_state_client)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception('Threw an exception')
        return False
