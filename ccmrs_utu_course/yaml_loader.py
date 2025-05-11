import yaml
import numpy as np

class ParamLoader():
    def __init__(self, param_file):
        # Load parameters from yaml file
        with open(param_file, 'r') as stream:
            self.param_dict = yaml.safe_load(stream)

        self.Ts = self.param_dict['Ts']
        self.Tmax = self.param_dict['t_max']
        self.ell = self.param_dict['look_ahead_ell']

        self.LIDAR_Ts = self.param_dict['LIDAR_Ts']

        self.parse_obstacles()

    def parse_obstacles(self):
        self.obstacles={}
        if 'obstacles' in self.param_dict:
            for key in self.param_dict['obstacles']:
                self.obstacles[key] = np.array(self.param_dict['obstacles'][key], dtype=float)


class ScenarioLoader():

    def __init__(self, scenario_file):
        with open(scenario_file, 'r') as stream:
            self.scenario_dict = yaml.safe_load(stream)

        self.list_robot_ID = self.scenario_dict['robot_ID']
        self.robot_num = len(self.scenario_dict['robot_ID'])

        self.parse_formation()


    def parse_formation(self):
        self.form_param = {}
        self.init_pos = {}
        self.grouping = {} # reverse lookup table on which formation a robot belong

        # Store param for each formation group
        for form_id in self.scenario_dict['formation']:
            param = self.scenario_dict['formation'][form_id]
            # Store formation distance (A matrix) and tolerance (epsilon value)
            self.form_param[form_id] = {
                'ids': np.array(param['member'], dtype=int),
                'A': np.array(param['distance'], dtype=float),
                'eps': param['tolerance'],
            }

            # Parse initial position
            form_cent = np.array(param['init_cent'], dtype=float)
            self.offset = np.array(param['cent_offset'], dtype=float)
            for i in range(len(param['member'])):
                robot_id = param['member'][i]
    
                self.init_pos[robot_id] = form_cent + self.offset[i]
                self.grouping[robot_id] = form_id



    def get_pairs_form_param(self, robot_id):
        form_id = self.grouping[robot_id]
        i = np.where(self.form_param[form_id]['ids'] == robot_id)[0][0]

        ids = self.form_param[form_id]['ids']
        A = self.form_param[form_id]['A']
        eps = self.form_param[form_id]['eps']

        pair_param = {}
        for j in range(len(ids)):
            if (i != j) and (A[i,j] > 0.):
                pair_param[ids[j]] = {'dist_ij':A[i,j], 'tol':eps}


        hull = self.form_param[form_id]['hull']
        # Find index of hull neighbors
        a = np.where(hull[i, :] < 0)[0][0]
        b_arr = np.where(hull[i, :] > 0)[0]
        b = b_arr[0] if len(b_arr) > 0 else a + 0
        # NOTE: by default a and b should be within neigh_q

        return pair_param, ids[a], ids[b]


