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
from .visualizer import PlotVisualizer, ProcessDataLog
from .main_controller import Estimation, calc_detected_pos

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

        # Initiate visualization for animating the robot movement
        self.plot_vis = PlotVisualizer(param, scenario)
        data = ProcessDataLog(param, scenario)

        # register and draw static obstacles from scenario settings
        for key in param.obstacles:
            obj_vertices = param.obstacles[key]
            self.plot_vis.ax_2D.plot(obj_vertices[:, 0], obj_vertices[:, 1], 'k')

        # Initiate the estimation to STORE DATA for each robot
        self.robot_est = {}
        for id in scenario.list_robot_ID:
            self.robot_est[id] = Estimation(id, param)


        # DEFINE SUBSCRIBER
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


        # Set timer for controller loop in each iteration
        self.ROS_RATE = round(1/param.Ts)
        self.Ts = 0.05 # 20Hz
        self.sim_timer = self.create_timer(self.Ts, self.vis_loop)
        self.it = 0
        self.start_t = self.time()
        self.check_t = self.time()

    def time(self):
        """Returns the current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9


    def posc_callback(self, msg, index): 
        self.robot_est[index].update_state_reading(
            np.array([msg.x, msg.y, 0]), msg.theta
        )

    def scan_LIDAR_callback(self, msg, index): 
        if self.robot_est[index].pos is not None:
            self.robot_est[index].update_range_sensors( np.array(msg.ranges) )
        # else: no position data yet


    # MAIN LOOP VISUALIZER
    def vis_loop(self):

        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):  # Add 10% extra margin
            self.get_logger().info(
                'WARNING loop rate is slower than expected. Period (ms): {:0.2f}'.format(diff * 1000))
        self.check_t = now

        # THE PART BELOW REPLICATE THE update FUNCTION IN PlotVisualizer
        # BUT ONLY THE ONE THAT IS BEING USED
        # ------------------------------------------------------------------------------
        # Visualization update
        for i in self.robot_est:
            self.plot_vis.canvas.plot_robot_pos(i, self.robot_est[i].pos, self.robot_est[i].theta)

            sensed_obs = self.robot_est[i].obs_pos
            if self.plot_vis.SHOW_LIDAR_DETECTION and sensed_obs is not None:
                try: # update existing plot
                    self.plot_vis.pl_sens[i].set_data(sensed_obs[:, 0], sensed_obs[:, 1])
                except: # Initiate the first time
                    self.plot_vis.pl_sens[i], = self.plot_vis.ax_2D.plot(sensed_obs[:, 0], sensed_obs[:, 1], '.', color=self.plot_vis.canvas._colorList[i])


        if self.plot_vis.SHOW_COMMUNICATION:
            self.plot_vis.plot_bidirectional_communication(self.robot_est)


        elapsed_time = (now - self.start_t)
        # self.plot_vis.canvas.plot_time(now)
        self.plot_vis.draw(elapsed_time)

        # ------------------------------------------------------------------------------



def main(args=None):
    ROS_NODE_NAME = 'mrs_visualizer'

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
