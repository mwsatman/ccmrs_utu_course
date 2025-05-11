#!/usr/bin/python3
import rclpy, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import LaserScan

from functools import partial

import os
import numpy as np
from .yaml_loader import ParamLoader, ScenarioLoader
from .main_controller import Estimation, Controller

# current_path = os.path.dirname(os.path.abspath(__file__)) + '/' 
# If the above code broke, use absolute path
current_path = "/home/mwatma/ros2_ws/src/ccmrs_utu_course/ccmrs_utu_course/"

param_file = "sim_setup.yaml"
scenario_file = "scenario_demo.yaml"

class Computation(Node):
    def __init__(self, ROS_NODE_NAME):
        super().__init__(ROS_NODE_NAME)
        
        self.declare_parameter('robot_ID', 0)
        robot_index = self.get_parameter('robot_ID').get_parameter_value().integer_value
        # self.get_logger().info('Hello robot %d!' % robot_index)

        # Load parameters from yaml file
        param = ParamLoader(current_path + param_file)
        scenario = ScenarioLoader(current_path + scenario_file)

        # Initialize Controller
        # Initiate the estimation and controller for each robot
        self.id = robot_index
        # for id in scenario.list_robot_ID:
        self.robot_est_i = Estimation(robot_index, param)
        self.robot_cont_i = Controller(robot_index, scenario)

        # for robot_index in scenario.list_robot_ID:
        tb_name = f'tb4_{robot_index}'

        # Create pose center subscribers
        self.get_logger().info(f'Creating pos center subscriber /{tb_name}/pos')
        self.pos_sub = self.create_subscription(Pose2D,
                                                f'/{tb_name}/pos',
                                                partial(self.posc_callback, index=robot_index),
                                                qos_profile=qos_profile_sensor_data)

        # Create LiDAR subscribers
        self.get_logger().info(f'Creating LiDAR data subscriber: /{tb_name}/scan')
        self.create_subscription(LaserScan,
                                 f'/{tb_name}/scan',
                                 partial(self.scan_LIDAR_callback, index=robot_index),
                                 qos_profile=qos_profile_sensor_data)


        self.cmd_vel_i = Twist()
        # Create cmd_vel publisher
        self.get_logger().info(f'Creating Twist cmd_vel publisher: /{tb_name}/cmd_vel')
        self.cmd_vel_pubs_i = self.create_publisher(Twist, '/{}/cmd_vel'.format(tb_name), 1)



        # Subscribe to Neighbour's data 
        # DEFINE SUBSCRIBER
        for robot_index in self.robot_est_i.neigh_ids:
            tb_name = f'tb4_{robot_index}'

            # Create pose center subscribers
            self.get_logger().info(f'Neighbour - Creating pos center subscriber /{tb_name}/pos')
            self.create_subscription(Pose2D,
                                     f'/{tb_name}/pos',
                                     partial(self.neigh_posc_callback, index=robot_index),
                                     qos_profile=qos_profile_sensor_data)


        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = param.Ts
        self.sim_timer = self.create_timer(self.Ts, self.control_loop)
        self.it = 0
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    # UPDATE SELF DATA
    def posc_callback(self, msg, index): 
        self.robot_est_i.update_state_reading(np.array([msg.x, msg.y, 0]), 
                                                msg.theta)

    def scan_LIDAR_callback(self, msg, index): 
        self.robot_est_i.update_range_sensors(np.array(msg.ranges))


    # UPDATE NEIGHBOUR DATA
    def neigh_posc_callback(self, msg, index): 
        self.robot_est_i.update_neigh_pose( index,
            np.array([msg.x, msg.y, 0]), msg.theta
        )

    def neigh_form_est_callback(self, msg, index): 
        self.robot_est_i.update_neigh_form_cent(index, np.array([msg.x, msg.y, 0]))


    # MAIN LOOP SIMULATOR
    def control_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now


        # COMPUTE CONTROL INPUT and PUBLISH CMD_VEL

        # Controller part
        if self.robot_est_i.obs_pos is not None:
            vel_command = self.robot_cont_i.compute_control_input(self.robot_est_i)
            lin_vel, ang_vel = self.robot_cont_i.si_to_unicycle(vel_command, 
                                                                    self.robot_est_i.theta, 
                                                                    self.robot_est_i.look_ahead_dist)

            # Sending command data
            self.cmd_vel_i.linear.x = lin_vel
            self.cmd_vel_i.angular.z = ang_vel
            self.cmd_vel_pubs_i.publish( self.cmd_vel_i )



def main(args=None):
    ROS_NODE_NAME = 'mrs_controller'

    print(args)

    rclpy.init(args=args)
    node = Computation(ROS_NODE_NAME)
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
