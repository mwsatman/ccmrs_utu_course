# ccmrs_utu_course
Code repository for the course: Cooperative Control of Multi-Robot System 

Each of the file describes the components for easier separation in python simulation, ROS simulation, and real-robot implementation. Thus, ease the process of transition from proof-of-concept to verification

- yaml files: contain simulation-specific setups and scenario-specific parameters
- yaml_loader: parse the yaml files and store them into the designated class for easier access and categorization
- simulator.py: main calculation to model the movement of the robots based on the control input
- visualizer.py: basic encapsulation of visualizing the movement and range-sensor measurement of the robots
- main_controller.py: contain the encapsulation of estimation and controller calculation for each robot

- simulation2D.py: main function to run the python simulator

You can run the simulator for python script via `python3 simulation2D.py`


## Setup in ROS2

- remember to source the ros setup, e.g., via `source /opt/ros/jazzy/setup.bash`
- clone this directory inside the `src` folder in your workspace, e.g., `~/ros2_ws/src`
- rebuild the workspace via `colcon build --symlink-install` inside `~/ros2_ws/`
- source the compiled package `source install/setup.bash`
- run the simulator via `ros2 run ccmrs_utu_course ROS2_sim_launch.py`

For further implementation on turtlebot4, follow the instruction in Moodle.