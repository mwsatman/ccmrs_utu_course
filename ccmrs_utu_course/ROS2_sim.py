#!/usr/bin/python3
import rclpy, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose2D, Twist

from functools import partial


import os
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


        # Initialize Robot and Range Finder
        self.robot_list = spawn_robots(scenario, param)

        # DEFINE SUBSCRIBER & PUBLISHER
        self.pose_pubs = {}
        self.scan_pubs = {}        
        
        self.all_pose = {}
        self.all_cmd_vel = {}

        for robot_index in scenario.list_robot_ID:
            tb_name = f'tb4_{robot_index}'

            # Create cmd_vel subscribers
            self.get_logger().info(f'Creating Twist cmd_vel subscriber: /{tb_name}/cmd_vel')
            self.create_subscription(Twist,
                                     f'/{tb_name}/cmd_vel',
                                     partial(self.cmd_vel_callback, index=robot_index),
                                     qos_profile=qos_profile_sensor_data)

            self.all_pose[robot_index] = Pose2D()

            # create Pose2D publisher
            self.get_logger().info(f'Creating Pose2D publisher: /{tb_name}/pos')
            self.pose_pubs[tb_name] = self.create_publisher(Pose2D, '/{}/pos'.format(tb_name), 1)


        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = param.Ts
        self.sim_timer = self.create_timer(self.Ts, self.sim_loop)
        self.it = 0
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def cmd_vel_callback(self, msg, index): 
        self.all_cmd_vel[index] = msg
        self.robot_list[index].set_input_unicycle(msg.linear.x, msg.angular.z)


    # MAIN LOOP SIMULATOR
    def sim_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        # Showing Time Stamp
        if (self.it > 0) and (self.it % self.ROS_RATE == 0):
            t = self.it * self.Ts
            self.get_logger().info('Simulation t = {}s.'.format(t))
        self.it += 1

        # Update Simulation and PUBLISH each robot TF and sensor reading
        for id, robot in self.robot_list.items():
            robot.update()
            # Sensor data
            self.all_pose[id].x = robot.state["pos"][0]
            self.all_pose[id].y = robot.state["pos"][1]
            self.all_pose[id].theta = robot.state["theta"]

            self.pose_pubs[f"tb4_{id}"].publish( self.all_pose[id] )



def main(args=None):
    ROS_NODE_NAME = 'mrs_simulator'

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
