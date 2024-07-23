import argparse
import sys
import time

from google.protobuf import wrappers_pb2

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import arm_command_pb2, estop_pb2, robot_command_pb2, synchronized_command_pb2
from bosdyn.client.estop import EstopClient
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.util import duration_to_seconds


def verify_estop(robot):
    """Verify the robot is not estopped"""

    client = robot.ensure_client(EstopClient.default_service_name)
    if client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
        error_message = 'Robot is estopped. Please use an external E-Stop client, such as the ' \
                        'estop SDK example, to configure E-Stop.'
        robot.logger.error(error_message)
        raise Exception(error_message)


def make_robot_command(arm_joint_traj):
    """ Helper function to create a RobotCommand from an ArmJointTrajectory.
        The returned command will be a SynchronizedCommand with an ArmJointMoveCommand
        filled out to follow the passed in trajectory. """

    joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_traj)
    arm_command = arm_command_pb2.ArmCommand.Request(arm_joint_move_command=joint_move_command)
    sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
    arm_sync_robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=sync_arm)
    return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)


def print_feedback(feedback_resp, logger):
    """ Helper function to query for ArmJointMove feedback, and print it to the console.
        Returns the time_to_goal value reported in the feedback """
    joint_move_feedback = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
    logger.info(f'  planner_status = {joint_move_feedback.planner_status}')
    logger.info(
        f'  time_to_goal = {duration_to_seconds(joint_move_feedback.time_to_goal):.2f} seconds.')

    # Query planned_points to determine target pose of arm
    logger.info('  planned_points:')
    for idx, points in enumerate(joint_move_feedback.planned_points):
        pos = points.position
        pos_str = f'sh0 = {pos.sh0.value:.3f}, sh1 = {pos.sh1.value:.3f}, el0 = {pos.el0.value:.3f}, ' \
                  f'el1 = {pos.el1.value:.3f}, wr0 = {pos.wr0.value:.3f}, wr1 = {pos.wr1.value:.3f}'
        logger.info(f'    {idx}: {pos_str}')
    return duration_to_seconds(joint_move_feedback.time_to_goal)


def ribbon_cut(config, robot, command_client, robot_state_client):
   
    stow = RobotCommandBuilder.arm_stow_command()
   
    # Issue the command via the RobotCommandClient
    stow_command_id = command_client.robot_command(stow)
   
    robot.logger.info('Stow command issued.')
    block_until_arm_arrives(command_client, stow_command_id, 3.0)
   
    # First point position
    
    # Arm Deploy Position
    sh0 = 0.000141143798828125
    sh1 = -0.8990693092346191
    el0 = 1.7974470853805542
    el1 = 0.0020961761474609375
    wr0 = -0.896080732345581
    wr1 = -0.004385709762573242
    
    # First point time (seconds)
    first_point_t = 2.0
   
    # Build the proto for the trajectory point.
    traj_point1 = RobotCommandBuilder.create_arm_joint_trajectory_point(
        sh0, sh1, el0, el1, wr0, wr1, first_point_t)
   
    # Second point position
    
    # Point above cutter
    sh0 = 2.9801864624023438
    sh1 = -1.64955472946167
    el0 =  1.8488434553146362
    el1 = 0.03802609443664551
    wr0 = 1.1939955949783325
    wr1 = -1.6041088104248047
    
    # First point time (seconds)
    second_point_t = 4.0
   
    # Build the proto for the trajectory point.
    traj_point2 = RobotCommandBuilder.create_arm_joint_trajectory_point(
        sh0, sh1, el0, el1, wr0, wr1, second_point_t)
    
   
    # Point at Cutter Grasp
    sh0 = 3.0554916858673096
    sh1 = -1.499003291130066
    el0 = 1.9619585275650024
    el1 = 0.037596940994262695
    wr0 = 1.034972906112671
    wr1 = -1.623246669769287
    
    
    # First point time (seconds)
    third_point_t = 5.0
   
    # Build the proto for the trajectory point.
    traj_point3 = RobotCommandBuilder.create_arm_joint_trajectory_point(
        sh0, sh1, el0, el1, wr0, wr1, third_point_t)
   
    max_vel = wrappers_pb2.DoubleValue(value=2.5)
    max_acc = wrappers_pb2.DoubleValue(value=15)
   
    # Build up a proto.
    arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point1, traj_point2],
                                                        maximum_velocity=max_vel,
                                                        maximum_acceleration=max_acc)
    # Make a RobotCommand
    command = make_robot_command(arm_joint_traj)
   
    # Send the request
    cmd_id = command_client.robot_command(command)
    robot.logger.info('Moving arm along 2-point joint trajectory.')
    
    # Query for feedback
    feedback_resp = command_client.robot_command_feedback(cmd_id)
    robot.logger.info('Feedback for Example 3: unmodified trajectory')
    time_to_goal = print_feedback(feedback_resp, robot.logger)
    # time.sleep(time_to_goal)
    
    gripper_command = RobotCommandBuilder.claw_gripper_open_command()
    gripper_command_id = command_client.robot_command(gripper_command)
    
    block_until_arm_arrives(command_client, gripper_command_id, 1.0)
    
    # Build up a proto.
    arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point2, traj_point3],
                                                        maximum_velocity=max_vel,
                                                        maximum_acceleration=max_acc)
    # Make a RobotCommand
    command = make_robot_command(arm_joint_traj)
   
    # Send the request
    cmd_id = command_client.robot_command(command)
    robot.logger.info('Moving arm along 2-point joint trajectory.')
   
    # Query for feedback to determine exactly what the planned trajectory is.
    feedback_resp = command_client.robot_command_feedback(cmd_id)
    robot.logger.info('Feedback for 2-point joint trajectory')
    print_feedback(feedback_resp, robot.logger)
   
    # Wait until the move completes before powering off.
    block_until_arm_arrives(command_client, cmd_id, second_point_t + 2.0)
    gripper_command = RobotCommandBuilder.claw_gripper_close_command()
    gripper_command_id = command_client.robot_command(gripper_command)
    
    #print(block_until_arm_arrives(command_client, gripper_command_id, 1.0))
         
    # Arm Deploy Position
    sh0 = 0.07207608222961426
    sh1 = -0.9832170009613037
    el0 = 1.354470133781433
    el1 = -0.03290224075317383
    wr0 = -0.8805127143859863
    wr1 = -1.3600597381591797 
   
    # First point time (seconds)
    fourth_point_t = third_point_t + 2
   
   #Build the proto for the trajectory point.
    traj_point4 = RobotCommandBuilder.create_arm_joint_trajectory_point(
        sh0, sh1, el0, el1, wr0, wr1, fourth_point_t)
    
    # Build up a proto.
    arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point3, traj_point4],
                                                        maximum_velocity=max_vel,
                                                        maximum_acceleration=max_acc)
    # Make a RobotCommand
    command = make_robot_command(arm_joint_traj)
   
    # Send the request
    cmd_id = command_client.robot_command(command)
    robot.logger.info('Moving arm along 4-point joint trajectory.')
   
    # Query for feedback
    feedback_resp = command_client.robot_command_feedback(cmd_id)
    robot.logger.info('Feedback for Example 4: unmodified trajectory')
    time_to_goal = print_feedback(feedback_resp, robot.logger)
    # time.sleep(time_to_goal)
    
    # Arm Deploy Position
    sh0 = 0.05998110771179199
    sh1 = -0.2670102119445801
    el0 = 1.344882845878601
    el1 = -0.02303171157836914
    wr0 = -1.6429469585418701
    wr1 = -1.3885750770568848
   
    # fifth point time (seconds)
    fifth_point_t = fourth_point_t + 2
   
   #Build the proto for the trajectory point.
    traj_point5 = RobotCommandBuilder.create_arm_joint_trajectory_point(
        sh0, sh1, el0, el1, wr0, wr1, fifth_point_t)
    
    # Build up a proto.
    arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point4, traj_point5],
                                                        maximum_velocity=max_vel,
                                                        maximum_acceleration=max_acc)
    # Make a RobotCommand
    command = make_robot_command(arm_joint_traj)
   
    # Send the request
    cmd_id = command_client.robot_command(command)
    robot.logger.info('Moving arm along fifth-point joint trajectory.')
   
    # Query for feedback
    feedback_resp = command_client.robot_command_feedback(cmd_id)
    robot.logger.info('Feedback for Example 5: unmodified trajectory')
    time_to_goal = print_feedback(feedback_resp, robot.logger)
    time.sleep(time_to_goal)
    