#!/usr/bin/python3
import rclpy, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan

from functools import partial

import os
import numpy as np
from .yaml_loader import ParamLoader, ScenarioLoader
from .simulator import Sensor, spawn_robots

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

        # Initialize Range Finder
        self.robot_list = scenario.list_robot_ID
        self.sensors = Sensor(scenario)

        # register obstacles from scenario settings
        for key in param.obstacles:
            obj_vertices = param.obstacles[key]
            self.sensors.eval_env.register_obstacle_bounded(key, obj_vertices)

        # DEFINE SUBSCRIBER & PUBLISHER
        self.all_scan = {}
        self.scan_pubs = {}        

        for robot_index in scenario.list_robot_ID:
            tb_name = f'tb4_{robot_index}'

            # Create pose center subscribers
            self.get_logger().info(f'Creating pos center subscriber /{tb_name}/pos')
            self.pos_sub = self.create_subscription(Pose2D,
                                                    f'/{tb_name}/pos',
                                                    partial(self.posc_callback, index=robot_index),
                                                    qos_profile=qos_profile_sensor_data)

            self.all_scan[robot_index] = LaserScan()

            # create Pose2D publisher
            self.get_logger().info(f'Creating LaserScan publisher: /{tb_name}/scan')
            self.scan_pubs[tb_name] = self.create_publisher(LaserScan, '/{}/scan'.format(tb_name), 1)


        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = param.LIDAR_Ts
        self.sim_timer = self.create_timer(self.Ts, self.sim_loop)
        self.it = 0
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def posc_callback(self, msg, index): 
        self.sensors.update_robot_i(index, np.array([msg.x, msg.y, 0]), msg.theta)


    # MAIN LOOP SIMULATOR
    def sim_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        # Sensor data
        for id in self.robot_list:
            self.all_scan[id].ranges = self.sensors.get_range_measurement(id)

        # PUBLISH each robot TF and sensor reading
        for id in self.robot_list:
            self.scan_pubs[f"tb4_{id}"].publish( self.all_scan[id] )



def main(args=None):
    ROS_NODE_NAME = 'mrs_sensors'

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
