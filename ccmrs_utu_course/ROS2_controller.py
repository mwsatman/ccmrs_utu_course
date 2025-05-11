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
        
        # Load parameters from yaml file
        param = ParamLoader(current_path + param_file)
        scenario = ScenarioLoader(current_path + scenario_file)

        # Initialize Controller
        # Initiate the estimation and controller for each robot
        self.robot_est, self.robot_cont = {}, {}
        for id in scenario.list_robot_ID:
            self.robot_est[id] = Estimation(id, param)
            self.robot_cont[id] = Controller(id, scenario)


        # DEFINE SUBSCRIBER & PUBLISHER        
        self.neigh_cent = {} # SIMPLIFIED COMMUNICATION EXCHANGE FOR NOW
        self.neigh_pos = {} # SIMPLIFIED COMMUNICATION EXCHANGE FOR NOW
        self.neigh_theta = {} # SIMPLIFIED COMMUNICATION EXCHANGE FOR NOW

        self.cmd_vel_pubs = {}
        self.all_cmd_vel = {}

        self.form_est_pose_pubs = {}
        self.all_form_est_pose = {}

        for robot_index in scenario.list_robot_ID:
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


            self.all_cmd_vel[robot_index] = Twist()
            # Create cmd_vel publisher
            self.get_logger().info(f'Creating Twist cmd_vel publisher: /{tb_name}/cmd_vel')
            self.cmd_vel_pubs[tb_name] = self.create_publisher(Twist, '/{}/cmd_vel'.format(tb_name), 1)


        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = param.Ts
        self.sim_timer = self.create_timer(self.Ts, self.control_loop)
        self.it = 0
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def posc_callback(self, msg, index): 
        self.robot_est[index].update_state_reading(np.array([msg.x, msg.y, 0]), 
                                                msg.theta)

    def scan_LIDAR_callback(self, msg, index): 
        self.robot_est[index].update_range_sensors(np.array(msg.ranges))


    # MAIN LOOP SIMULATOR
    def control_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        # Update communication from other robots, later merge into callback
        for id in self.all_cmd_vel:
            for neigh_id in self.robot_est[id].neigh_ids:
                # update neighbour's pose 
                if neigh_id in self.neigh_pos:
                    self.robot_est[id].update_neigh_pose(neigh_id, self.neigh_pos[neigh_id], self.neigh_theta[neigh_id])
                # update neighbour's centroid estimate
                if neigh_id in self.neigh_cent:
                    self.robot_est[id].update_neigh_form_cent(neigh_id, self.neigh_cent[neigh_id])


        # COMPUTE CONTROL INPUT and PUBLISH CMD_VEL
        for id in self.all_cmd_vel:
            if self.robot_est[id].obs_pos is not None:
                # Controller part
                vel_command = self.robot_cont[id].compute_control_input(self.robot_est[id])
                lin_vel, ang_vel = self.robot_cont[id].si_to_unicycle(vel_command, 
                                                                    self.robot_est[id].theta, 
                                                                    self.robot_est[id].look_ahead_dist)

                # Sending command data
                self.all_cmd_vel[id].linear.x = lin_vel
                self.all_cmd_vel[id].angular.z = ang_vel
                self.cmd_vel_pubs[f"tb4_{id}"].publish( self.all_cmd_vel[id] )

                # Send communication to other robots
                self.neigh_pos[id] = self.robot_est[id].pos
                self.neigh_theta[id] = self.robot_est[id].theta


def main(args=None):
    ROS_NODE_NAME = 'mrs_controller'

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
