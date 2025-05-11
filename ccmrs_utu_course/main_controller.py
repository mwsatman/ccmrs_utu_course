import numpy as np

def calc_lahead_pos(pos, theta, ell):
    return np.array([pos[0] + ell*np.cos(theta), 
                     pos[1] + ell*np.sin(theta), 
                     pos[2] ])    

def calc_detected_pos(range_data, pos, theta, beam_angles):
    all_detected_pos = np.zeros((len(beam_angles), 3))
    sensing_angle_rad = theta + beam_angles
    all_detected_pos[:, 0] = pos[0] + range_data * np.cos(sensing_angle_rad)
    all_detected_pos[:, 1] = pos[1] + range_data * np.sin(sensing_angle_rad)
    return all_detected_pos



class Estimation():
    def __init__(self, robot_ID, param_dict):
        self.robot_ID = robot_ID

        # SENSOR-BASED Variables
        self.pos = None
        self.theta = None
        self.range_data = None
        self.range_pos = None
        self.obs_pos = None

        self.Ts = param_dict.Ts
        self.look_ahead_dist = param_dict.ell
        self.lahead_pos = None
        self.del_lahead = np.zeros(3) # shift of l_ahead pos over time

        sensing_resolution = 360 # normal LiDAR on turtlebot
        self.beam_angles = np.linspace(0., 2*np.pi, num=sensing_resolution, endpoint=False)

        # CONSENSUS-BASED & COMMUNICATION EXCHANGE Variables
        self.neigh_ids = []
        self.neighbours_data = {} # store neighbours information
        self.cent_est = None
        self.vec_ci = np.zeros(3) # initial estimate of shift from centroid to robot position
        self.vel_command = np.zeros(3)


    # SENSOR-BASED ESTIMATION
    # ----------------------------------------------------------------------------------
    def update_state_reading(self, pos, theta):
        # receiving accurate position and theta directly
        # e.g., from simulator or motion capture
        self.pos = pos
        self.theta = theta
        new_lahead = calc_lahead_pos(pos, theta, self.look_ahead_dist)
        if self.lahead_pos is not None:
            self.del_lahead = new_lahead - self.lahead_pos
        self.lahead_pos = new_lahead
        # Initiate centroid estimation if needed
        if self.cent_est is None: self.cent_est = pos - self.vec_ci 

    def update_range_sensors(self, range_data):
        self.range_data = range_data
        # compute the position of the detected obstacles
        self.range_pos = calc_detected_pos(range_data, self.pos, self.theta, self.beam_angles)
        # filter valid obs
        self.obs_pos = self.range_pos[range_data > 0.05]
        # self.obs_pos = self.range_pos[range_data < 0.99 * max_value]


    # CONSENSUS-BASED ESTIMATION & COMMUNICATION EXCHANGE
    # ----------------------------------------------------------------------------------
    def update_neigh_pose(self, robot_id, pos, theta):
        neigh_lahead = calc_lahead_pos(pos, theta, self.look_ahead_dist)
        if robot_id not in self.neighbours_data:
            cent_est = neigh_lahead # initiate estimation with neighbours lahead pos
        else:
            cent_est = self.neighbours_data[robot_id]['cent']
        self.neighbours_data[robot_id] = {'pos': pos, 'theta':theta, 'lahead': neigh_lahead, 'cent': cent_est}

    def update_neigh_form_cent(self, robot_id, cent_est):
        if robot_id in self.neighbours_data:
            self.neighbours_data[robot_id]['cent'] = cent_est
        # else: uninitialized data (skip for now)



class Controller():
    def __init__(self, robot_ID, scenario_dict):
        self.robot_ID = robot_ID

        ## ------------------------------------
        # INITIATE ALL VARIABLES FOR CONTROLLER
        ## ------------------------------------




    def compute_control_input(self, estimation_dict):
        current_pos = estimation_dict.lahead_pos

        ## ------------------------------------
        # CONTROLLER CALCULATION
        ## ------------------------------------
        # Compute nominal control # TODO
        vx = 0.1
        vy = 0.1
        u_nom = np.array([vx, vy, 0])

        # Implement safety controller # TODO
        vel_command = u_nom

        # return the resulting control input
        estimation_dict.vel_command = vel_command
        return vel_command



    @staticmethod
    def si_to_unicycle(u, theta, ell):
        vel_lin = u[0]*np.cos(theta) + u[1]*np.sin(theta)
        vel_ang = (- u[0]*np.sin(theta) + u[1]*np.cos(theta))/ell
        return vel_lin, vel_ang
