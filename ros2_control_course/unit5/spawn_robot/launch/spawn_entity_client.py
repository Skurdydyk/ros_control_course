#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import xacro


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    # Robot model: configure as needed
    robot_model_package = sys.argv[1]
    robot_model_relative_path = sys.argv[2]
    robot_model_file = sys.argv[3]

    xacro_file = os.path.join(get_package_share_directory(
        robot_model_package), robot_model_relative_path, robot_model_file)
    assert os.path.exists(
        xacro_file), "The file "+str(robot_model_file)+" doesnt exist in "+str(xacro_file)

    install_dir = get_package_prefix(robot_model_package)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    content = robot_desc

    """ debug
    # Count the arguments
    arguments = len(sys.argv) - 1
    print("The spawn entity script is called with %i arguments" % (arguments))
    print(f"Arguments count: {len(sys.argv)}")
    for i, arg in enumerate(sys.argv):
        print(f"Argument {i:>6}: {arg}")
    """

    # Set data for request
    req = SpawnEntity.Request()
    req.name = sys.argv[4]
    req.xml = content
    req.robot_namespace = ""
    req.reference_frame = "world"
    req.initial_pose.position.x = float(sys.argv[5])
    req.initial_pose.position.y = float(sys.argv[6])
    req.initial_pose.position.z = float(sys.argv[7])

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Gazebo '/spawn_entity' service not available, waiting ...")

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
