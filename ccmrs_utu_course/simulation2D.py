from simulator import Sensor, Communication, spawn_robots
from visualizer import PlotVisualizer, ProcessDataLog
from main_controller import Estimation, Controller
from yaml_loader import ParamLoader, ScenarioLoader
import os

param_file = "sim_setup.yaml"
scenario_file = "scenario_demo.yaml"

if __name__ == "__main__":

    # Load parameters from yaml file
    current_path = os.path.dirname(os.path.abspath(__file__)) + '/'
    param = ParamLoader(current_path + param_file)
    scenario = ScenarioLoader(current_path + scenario_file)


    # Initialize Robot and Range Finder
    robot_list = spawn_robots(scenario, param)
    sensors = Sensor(scenario)
    comm = Communication()

    # Initiate visualization for animating the robot movement
    plot_vis = PlotVisualizer(param, scenario)
    data = ProcessDataLog(param, scenario)

    # register and draw static obstacles from scenario settings
    for key in param.obstacles:
        obj_vertices = param.obstacles[key]
        sensors.eval_env.register_obstacle_bounded(key, obj_vertices)
        plot_vis.ax_2D.plot(obj_vertices[:, 0], obj_vertices[:, 1], 'k')

    # Initiate the estimation and controller for each robot
    robot_est, robot_cont = {}, {}
    for id in scenario.list_robot_ID:
        robot_est[id] = Estimation(id, param)
        robot_cont[id] = Controller(id, scenario)

        # Initiate self state value
        robot = robot_list[id]
        robot_est[id].update_state_reading(robot.state["pos"], robot.state['theta'])            
        

    # Main loop to calculate robot's movement
    time = 0.0
    while time < param.Tmax:
        # Showing Time Stamp
        if (round(time*100) % 100 == 0):
            if time < param.Tmax: print(f'simulating t = {time:.2f}s.')


        for id, robot in robot_list.items():
            # simulate the process of communication exchange
            comm.parse_msg_for_j(robot_est[id])

            # Sensor data
            # simulate the process of measurement readings & estimation
            robot_est[id].update_state_reading(robot.state["pos"], robot.state['theta'])
            robot_est[id].update_range_sensors(sensors.get_range_measurement(id))

            # Controller part
            vel_command = robot_cont[id].compute_control_input(robot_est[id])

            # Sending command
            robot.set_input_lookahead(vel_command)
            # exchange messages
            comm.send_msg_to_neigh(robot_est[id])


        # Draw the robot position 
        data.store_datas(time, robot_est, robot_cont)
        plot_vis.update(time, robot_est, robot_cont, data)
        # Update simulation after all robots done sending command
        for id, robot in robot_list.items():
            robot.update()
            sensors.update_robot_i(id, robot.state['pos'], robot.state['theta'])
        # Exchange communication
        comm.exchange_communication()

        time += param.Ts
