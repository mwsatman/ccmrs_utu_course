import numpy as np

def spawn_robots(scenario_dict, param_dict):
    robot_list = {}
    for id in scenario_dict.list_robot_ID:
        robot_list[id] = Unicycle(param_dict.Ts, 
                                  init_pos=scenario_dict.init_pos[id], init_theta=0., 
                                  look_ahead_dist=param_dict.ell, robot_ID=id)
    return robot_list


class Dynamic:
    """
    Robot's dynamical model.
    It can consider a simpler kinematic model or a more complex one.
    This class is a template, to implement continuous model in digital via Forward Euler approach.
    A new class can inherit this class and adjust the compute_dot_state and/or add new methods.
    """

    def __init__(self, dt):
        """
        Initialize variables to contain the state and input.
        Each state and input are defined inside a dictionary,
        to allow different naming convention between each model.

        :param dt: (default) time sampling for update
        """
        self.model_name = self.__class__.__name__
        self.Ts = dt
        self.state = {}
        self.dot_state = {}
        self.input = {}
        self.model_name = None

    def compute_dot_state(self):
        """
        Define the robot's model as a relation of input and state to dot_state
        for example,
            self.dot_state["q"] = self.state["q"] + self.input["u"]
        """
        pass

    def update(self, Ts=None):
        """
        Update the next state based on Forward Euler computation.
        Allow computation with varying time sampling, but only this instance.
        If not defined again in the next step it default back to existing self.dt,
        for permanent changes of dt, directly set self.dt.

        :return:
        """
        dt = self.Ts if Ts is None else Ts

        # Compute next state (based on nominal model)
        self.compute_dot_state()

        # Increment from past to present state
        for k, v in self.state.items():
            self.state[k] = v + dt * self.dot_state[k]

        return self.state


class Unicycle(Dynamic):
    """
    A unicycle dynamic for a robot in planar plane (3 dimension with z=0).
    State: Position as pos = [q_x q_y q_z], orientation as theta = th
    Input: Linear velocity as linV = V, Angular Velocity as angV = omg
    Dynamics:
        dot(q) = [linV*cos(theta), linV*sin(theta), 0.]
        dot(theta) = angV
    """

    def __init__(self, dt, *,
                 init_pos=np.array([0., 0., 0.]), init_theta=0.,
                 init_linV=0., init_angV=0.,
                 robot_ID=None, look_ahead_dist=-0.1,
                 max_linV=-0.1, max_angV=-0.1):
        """
        :param dt: (default) time sampling for update
        :param init_pos: robot's initial position in numpy array
        :param init_vel: robot's initial velocity input in numpy array
        :param robot_ID: to identify different robot for multi-robot scenarios

        :param look_ahead_dist: set the point with a distance [m] ahead of robot,
        to transformation from world velocity input into linV and angV
        (look_ahead_dist > 0, enabling control of the look ahead point as a single integrator)

        :param dt:
        """

        super().__init__(dt)
        self.robot_ID = robot_ID

        self.state["pos"], self.state["theta"] = init_pos, init_theta
        self.input["linV"], self.input["angV"] = init_linV, init_angV
        self.dot_state["pos"] = np.array([0., 0., 0.])
        self.dot_state["theta"] = 0.

        self.look_ahead_dist = look_ahead_dist
        self.max_linV, self.max_angV = max_linV, max_angV

    def compute_dot_state(self):
        """
        Dynamics:
            dot(q) = [linV*cos(theta), linV*sin(theta), 0.]
            dot(theta) = angV
        """
        self.dot_state["pos"] = np.array([
            self.input["linV"] * np.cos(self.state["theta"]),
            self.input["linV"] * np.sin(self.state["theta"]),
            0.
        ])
        self.dot_state["theta"] = self.input["angV"]

    @staticmethod
    def impose_unicycle_saturation(linV, angV, max_linV, max_angV):
        """
        :param linV: float, linear velocity input
        :param angV: float, angular velocity input
        :param max_linV: float, maximum linear velocity input
        :param max_angV: float, maximum angular velocity input
        """
        if max_linV > 0. and max_angV > 0.:
            sat_linV, sat_angV = linV, angV
            if (max_linV > 0.) and (abs(linV) >= max_linV):
                sat_linV = max_linV * linV / abs(linV)
            if (max_angV > 0.) and (abs(angV) >= max_angV):
                sat_angV = max_angV * angV / abs(angV)
            return sat_linV, sat_angV
        else:
            return linV, angV
        
    def set_input_unicycle(self, linV, angV):
        """
        :param linV: float, linear velocity input
        :param angV: float, angular velocity input
        """
        linV, angV = self.impose_unicycle_saturation(linV, angV, self.max_linV, self.max_angV)
        self.input["linV"], self.input["angV"] = linV, angV

    def set_input_lookahead(self, vel):
        """
        Inverse Look up ahead Mapping (u_z remain 0.)
            linV = u_x cos(theta) + u_y sin(theta)
            angV = (- u_x sin(theta) + u_y cos(theta)) / l

        :param vel: velocity input in numpy array
        """
        assert (self.look_ahead_dist > 0.), \
            f"Input with lookahead is disabled. look_ahead_dist={self.look_ahead_dist} should be > 0."

        th = self.state["theta"]

        # do SI to unicycle conversion
        Ml = np.array([[1, 0], [0, 1 / self.look_ahead_dist]])
        Mth = np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])
        current_input = Ml @ Mth @ vel[:2]

        linV, angV = current_input[:2]
        linV, angV = self.impose_unicycle_saturation(linV, angV, self.max_linV, self.max_angV)

        self.input["linV"] = linV
        self.input["angV"] = angV


class Sensor():
    def __init__(self, scenario_dict):
        self.robot_pos = {}
        self.robot_theta = {}

        # Initiate the environment evaluator for range sensor
        self.eval_env = DetectObstacle2D()
        # Specification of the range sensor model
        sensing_resolution, self.max_value = 360, 3 # assuming 360 resolution with maximum 3m reading
        self.beam_angles = np.linspace(0., 2*np.pi, num=sensing_resolution, endpoint=False)

        # Register all robots and obstacles
        self.robot_rad = 0.1 
        for id in scenario_dict.list_robot_ID:
            self.robot_pos[id] = scenario_dict.init_pos[id]
            self.robot_theta[id] = 0.

            self.eval_env.register_obstacle_bounded(id, calc_robot_circ_bounds(self.robot_pos[id], self.robot_theta[id], self.robot_rad))


    def get_range_measurement(self, robot_ID):
        return self.eval_env.get_sensing_data(
                    self.robot_pos[robot_ID][0], self.robot_pos[robot_ID][1], self.robot_theta[robot_ID],
                    exclude=[robot_ID], beam_angles=self.beam_angles, max_distance=self.max_value, default_empty_val=0.)


    def update_robot_i(self, robot_ID, pos, theta):
        self.robot_pos[robot_ID] = pos
        self.robot_theta[robot_ID] = theta
        self.eval_env.register_obstacle_bounded(robot_ID, calc_robot_circ_bounds(pos, theta, self.robot_rad))    



def calc_robot_circ_bounds(pos, theta, robot_rad, sides=8):
    # Approximate robot bound as polygon with a given number of equal sides
    # Putting a higher number of sides making it closer to circle but add burden for obstacle detection
    robot_angle_bound = np.append(np.linspace(0., 2 * np.pi, num=sides, endpoint=False), 0) + np.pi / 8
    # Update robot shape to be used for range detection
    v_angles = robot_angle_bound + theta
    robot_shape = np.array([np.cos(v_angles), np.sin(v_angles), v_angles * 0]) * robot_rad
    robot_bounds = np.transpose(robot_shape + pos.reshape(3, 1))
    return robot_bounds

def calc_detected_pos(range_data, pos, theta, beam_angles):
    all_detected_pos = np.zeros((len(beam_angles), 3))
    sensing_angle_rad = theta + beam_angles
    all_detected_pos[:, 0] = pos[0] + range_data * np.cos(sensing_angle_rad)
    all_detected_pos[:, 1] = pos[1] + range_data * np.sin(sensing_angle_rad)
    return all_detected_pos


class DetectObstacle2D():

    def __init__(self):
        # Store the obstacle as line segments (x1, y1, x2, y2)
        self.__y1_min_y2, self.__x1_min_x2 = {}, {}
        self.__line_segment_2D = {}

    def register_obstacle_bounded(self, id, vertices):
        # store list of vertices that construct the obstacle into self.__line_segment
        # expect the vertices to be numpy array N x 3 
        # TODO: assert that the last vertex should be the same as the first
        new_line_segment = np.zeros((vertices.shape[0]-1, 4))
        new_line_segment[:,:2] = vertices[:-1,:2]
        new_line_segment[:,2:] = vertices[1:,:2]
        # store the data
        self.__line_segment_2D[id] = new_line_segment
        # self.__line_segment_2D = np.vstack((self.__line_segment_2D, new_line_segment))
        self.__update_basic_comp(id)

    def remove_obstacle_bounded(self, id):
        del self.__line_segment_2D[id]
        del self.__y1_min_y2[id]
        del self.__x1_min_x2[id]

    def __update_basic_comp(self, id):
        self.__y1_min_y2[id] = self.__line_segment_2D[id][:,1] - self.__line_segment_2D[id][:,3]
        self.__x1_min_x2[id] = self.__line_segment_2D[id][:,0] - self.__line_segment_2D[id][:,2]

    def get_sensing_data(self, posx, posy, theta_rad, exclude=[],
                         beam_angles=np.linspace(0., 2*np.pi, num=360, endpoint=False),
                         max_distance=10, default_empty_val = None):
        # The computation of detected obstacle will rely on the intersection 
        # between sensing's line-segment and obstacle's line-segment
        # The basic computation is following https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

        # Given the obstacle's line-segment denoted by (x1, y1) and (x2, y2)
        # and the sensing's line-segment denoted by (x3, y3) and (x4, y4), with
        # u = ( (x1-x3)(y1-y2) - (y1-y3)(x1-x2) ) / ( (x1-x2)(y3-y4) - (y1-y2)(x3-x4) ),
        # the obstacle is detected at (x3 + u(x4-x3), y3 + u(y4-y3))
        # if 0 <= u <= 1
        # So with (x3, y3) as the sensor's position then 
        # u is the ratio from the maximum sensing distance
        
        # number of sensing beam is m and number of obstacle line segment is n
        m = len(beam_angles)
        sensing_angle_rad = theta_rad + beam_angles
        m_x4_min_x3 = max_distance * np.cos( sensing_angle_rad )
        m_y4_min_y3 = max_distance * np.sin( sensing_angle_rad )

        # FIltering and collecting all line segments
        line_segment_2D = np.zeros((0,4)) 
        n_y1_min_y2 = np.zeros((0,1)) 
        n_x1_min_x2 = np.zeros(0) 
        for key in self.__line_segment_2D:
            if key not in exclude:
                # TODO: some selection for the nearest obstacle
                line_segment_2D = np.vstack((line_segment_2D, self.__line_segment_2D[key]))
                n_y1_min_y2 = np.append(n_y1_min_y2, self.__y1_min_y2[key])
                n_x1_min_x2 = np.append(n_x1_min_x2, self.__x1_min_x2[key])

        # Computing the intersections
        n = line_segment_2D.shape[0]
        n_0 = np.repeat(0., n)
        n_1 = np.repeat(1., n)

        n_x1_min_x3 = line_segment_2D[:,0] - np.repeat(posx, n)
        n_y1_min_y3 = line_segment_2D[:,1] - np.repeat(posy, n)

        # Loop over each sensing direction
        u_all = np.repeat(1., m)
        for i in range(m):
            # create repmat x3 and y3 for n_obs_lseg
            n_x3_min_x4 = - np.repeat( m_x4_min_x3[i], n )
            n_y3_min_y4 = - np.repeat( m_y4_min_y3[i], n )

            t_upper = (n_x1_min_x3 * n_y3_min_y4) - (n_y1_min_y3 * n_x3_min_x4)
            u_upper = (n_x1_min_x3 * n_y1_min_y2) - (n_y1_min_y3 * n_x1_min_x2)
            lower = (n_x1_min_x2 * n_y3_min_y4) - (n_y1_min_y2 * n_x3_min_x4)
            with np.errstate(divide='ignore'):
                t = t_upper / lower
                u = u_upper / lower

            t_idx = np.logical_and( t >= n_0, t <= n_1 )
            u_idx = np.logical_and( u >= n_0, u <= n_1 )
            idx = np.logical_and( t_idx, u_idx )
            if np.any(idx): u_all[i] = min( u[idx] )

        sensing_data = max_distance * u_all
        # Assign the default value for measurement with no object
        # e.g, the common LiDAR in turtlebot assign 0 value for no-detection
        if default_empty_val is not None:
            sensing_data[sensing_data > 0.99*max_distance] = default_empty_val

        return sensing_data

        # sensing_pos = np.array([[posx, posy],]*m)
        # sensing_pos[:,0] += u_all * m_x4_min_x3
        # sensing_pos[:,1] += u_all * m_y4_min_y3

        # return sensing_data, sensing_pos



class Communication():

    def __init__(self):

        self.incoming_buffer = {}
        self.outgoing_buffer = {}

    def send_msg_to_j(self, i, j, tag, msg):
        if j not in self.outgoing_buffer:
            self.outgoing_buffer[j] = []
        
        self.outgoing_buffer[j] += [{'fr':i, 'to':j, 'tag':tag, 'msg':msg}]

    def exchange_communication(self):
        self.incoming_buffer = self.outgoing_buffer.copy()
        self.outgoing_buffer = {} # reset outgoing buffer

    def get_msg_for_j(self, j):
        if j in self.incoming_buffer:
            return self.incoming_buffer[j]
        else:
            return []
    
    # ------------------------------------------------------------------------
    def send_msg_to_neigh(self, j_est_class):
        id = j_est_class.robot_ID
        for neigh_id in j_est_class.neigh_ids:
            self.send_msg_to_j(id, neigh_id, 'pose', {'pos':j_est_class.pos, 'theta':j_est_class.theta}) # MOCAP DATA
            self.send_msg_to_j(id, neigh_id, 'cent', j_est_class.cent_est) # Exchange estimation


    def parse_msg_for_j(self, j_est_class):
        id = j_est_class.robot_ID
        incoming_msg = self.get_msg_for_j(id)
        
        for msg in incoming_msg:
            neigh_id = msg['fr']

            if (msg['tag'] == 'pose') and (neigh_id in j_est_class.neigh_ids):
                pose = msg['msg']
                j_est_class.update_neigh_pose(neigh_id, pose['pos'], pose['theta'])

            elif (msg['tag'] == 'cent') and (neigh_id in j_est_class.neigh_ids):
                j_est_class.update_neigh_form_cent(neigh_id, msg['msg'])



