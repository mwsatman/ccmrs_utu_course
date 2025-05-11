import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np

class draw2DPointSI():
    """
    A class for plotting for multi-robots with single integrator (kinematic) model
    in 2D field (x,y plane).
    """

    def __init__(self, ax, *, field_x=None, field_y=None, pos_trail_nums=0):
        # The state should be n rows and 3 dimensional column (pos.X, pos.Y, and theta)
        # pos_trail_nums determine the number of past data to plot as trajectory trails
        self._ax = ax
        self._ax.set(xlabel="x [m]", ylabel="y [m]")
        self._ax.set_aspect('equal', adjustable='box', anchor='C')
        # Set field
        if field_x is not None:
            self._ax.set(xlim=(field_x[0] - 0.1, field_x[1] + 0.1))
        if field_y is not None:
            self._ax.set(ylim=(field_y[0] - 0.1, field_y[1] + 0.1))

        # plot placeholder for the position
        self._pl_pos = {}

        self._trail_num = pos_trail_nums
        if pos_trail_nums > 0:
            # Prepare buffer for the trail
            self._pl_trail = {}
            self._trail_data = {}

        # Plotting variables
        self._colorList = plt.rcParams['axes.prop_cycle'].by_key()['color']
        self._drawn_time = None

    def plot_robot_pos(self, key_id, pos):
        # Update data on existing plot
        if key_id in self._pl_pos:
            self._pl_pos[key_id].set_data([pos[0]], [pos[1]])

        # Initiate plot the first time
        else:
            # Adjust color
            color_id = key_id % (len(self._colorList))
            # Draw first position
            self._pl_pos[key_id], = self._ax.plot(pos[0], pos[1],
                                                  color=self._colorList[color_id],
                                                  marker='X', markersize=10)

        # Update the trail data
        self.update_trail(key_id, pos)


    def update_trail(self, key_id, pos):
        # Update data on existing plot
        if self._trail_num > 0:  # update trail data
            if key_id in self._trail_data:
                # roll the data, fill the new one from the top and then update plot
                self._trail_data[key_id] = np.roll(self._trail_data[key_id], self._trail_data[key_id].shape[1])
                self._trail_data[key_id][0, :] = pos
                self._pl_trail[key_id].set_data(self._trail_data[key_id][:, 0], self._trail_data[key_id][:, 1])

            else:
                # Adjust color
                color_id = key_id % (len(self._colorList))
                # use initial position to populate all matrix (pos_trail_nums-row x dim-col)
                # theta is not used for plotting the trail
                self._trail_data[key_id] = np.tile(pos, (self._trail_num, 1)).astype(float)

                # Plot the first trail data
                self._pl_trail[key_id], = self._ax.plot(
                    self._trail_data[key_id][:, 0], self._trail_data[key_id][:, 1],
                    '--', color=self._colorList[color_id])


    def plot_time(self, time):
        if self._drawn_time is None:
            # Display simulation time
            self._drawn_time = self._ax.text(0.78, 0.99,
                                             't = ' + f"{time:.2f}" + ' s', color='k', fontsize='large',
                                             horizontalalignment='left', verticalalignment='top',
                                             transform=self._ax.transAxes)
        else:
            self._drawn_time.set_text('t = ' + f"{time:.2f}" + ' s')



class draw2DUnicyle(draw2DPointSI):

    def __init__(self, ax, *, field_x=None, field_y=None, pos_trail_nums=0):
        super().__init__(ax, 
                         field_x=field_x, field_y=field_y, pos_trail_nums=pos_trail_nums)

    def plot_robot_pos(self, key_id, pos, theta):
        # Adjust color
        color_id = key_id % (len(self._colorList))
        self.__draw_icon( key_id, pos, theta, arrow_col=self._colorList[color_id])
        # Update the trail data
        self.update_trail(key_id, pos)

    def __draw_icon(self, key_id, pos, theta, arrow_col = 'b'): # draw mobile robot as an icon
        # Extract data for plotting
        px, py, th = pos[0], pos[1], theta
        # Basic size parameter
        scale = 1
        body_rad = 0.08 * scale # m
        wheel_size = [0.1*scale, 0.02*scale] 
        arrow_size = body_rad
        # left and right wheels anchor position (bottom-left of rectangle)
        thWh = [th+0., th+np.pi] # unicycle
        # thWh = [ (th + i*(2*np.pi/3) - np.pi/2) for i in range(3)] # for omnidirectional icon
        thWh_deg = [np.rad2deg(i) for i in thWh]
        wh_x = [ px - body_rad*np.sin(i) - (wheel_size[0]/2)*np.cos(i) + (wheel_size[1]/2)*np.sin(i) for i in thWh ]
        wh_y = [ py + body_rad*np.cos(i) - (wheel_size[0]/2)*np.sin(i) - (wheel_size[1]/2)*np.cos(i) for i in thWh ]
        # Arrow orientation anchor position
        ar_st= [px, py] #[ px - (arrow_size/2)*np.cos(th), py - (arrow_size/2)*np.sin(th) ]
        ar_d = (arrow_size*np.cos(th), arrow_size*np.sin(th))
        # initialized unicycle icon at the center with theta = 0
        if key_id not in self._pl_pos: # first time drawing
            self._pl_pos[key_id] = [None]*(2+len(thWh))
            self._pl_pos[key_id][0] = self._ax.add_patch( plt.Circle( (px, py), body_rad, color='#AAAAAAAA') )
            self._pl_pos[key_id][1] = self._ax.quiver( ar_st[0], ar_st[1], ar_d[0], ar_d[1], 
                scale_units='xy', scale=1, color=arrow_col, width=0.1*arrow_size)
            for i in range( len(thWh) ):
                self._pl_pos[key_id][2+i] = self._ax.add_patch( plt.Rectangle( (wh_x[i], wh_y[i]), 
                    wheel_size[0], wheel_size[1], angle=thWh_deg[i], color='k') )
        else: # update existing patch
            self._pl_pos[key_id][0].set( center=(px, py) )
            self._pl_pos[key_id][1].set_offsets( ar_st )
            self._pl_pos[key_id][1].set_UVC( ar_d[0], ar_d[1] )
            for i in range( len(thWh) ):
                self._pl_pos[key_id][2+i].set( xy=(wh_x[i], wh_y[i]) )
                self._pl_pos[key_id][2+i].angle = thWh_deg[i]


class PlotVisualizer():
    def __init__(self, param_dict, scenario_dict):

        # PARAMETER ON VISUALIZATION
        self.SHOW_LIDAR_DETECTION = True
        self.SHOW_CENTROID_ESTIMATION = False #True
        self.SHOW_COMMUNICATION = False #True
        self.SHOW_ROBOT_GOAL = False #True

        # TIME SERIES DATA
        self.SHOW_DISTANCE_MAINTENACE = False
        self.SHOW_MINH_FUNCTION = False

        self.list_ID = scenario_dict.list_robot_ID
        self.Tmax = param_dict.Tmax
        self.Ts = param_dict.Ts

        # self.time_series_window = param_dict.Tmax # in second
        self.time_series_window = 5. # in second


        # For now plot 2D with 2x2 grid space, to allow additional plot later on
        rowNum, colNum = 2, 2
        self.fig = plt.figure(figsize=(4 * colNum, 3 * rowNum), dpi=100)
        gs = GridSpec(rowNum, colNum, figure=self.fig)

        # Initiate the canvas for animating the robot movement
        self.ax_2D = self.fig.add_subplot(gs[0:2, 0:2])  # Always on
        self.canvas = draw2DUnicyle( self.ax_2D, field_x = [-2.5, 2.5], field_y = [-1.5, 1.5])

        # Save list of robots within each formation
        self.form_ids, self.form_A, self.form_tol = {}, {}, {}
        for f_id in scenario_dict.form_param:
            self.form_ids[f_id] = scenario_dict.form_param[f_id]['ids']
            self.form_A[f_id] = scenario_dict.form_param[f_id]['A']
            self.form_tol[f_id] = scenario_dict.form_param[f_id]['eps']

        self.pl_sens = {} # for self.SHOW_LIDAR_DETECTION
        self.pl_goal = {} # for self.SHOW_ROBOT_GOAL
        self.pl_cent = {} # for self.SHOW_CENTROID_ESTIMATION
        self.pl_comm = {} # for self.SHOW_COMMUNICATION

        self.pl_dist = {} # for self.SHOW_DISTANCE_MAINTENACE
        self.pl_minh = {} # for self.SHOW_MINH_FUNCTION


        self.tseries_colorList = plt.rcParams['axes.prop_cycle'].by_key()['color']

        if self.SHOW_DISTANCE_MAINTENACE:
            # Plot the distance between robots
            self.ax_dist = self.fig.add_subplot(gs[0, 2])
            self.ax_dist.set(xlabel="t [s]", ylabel="distance [m]")
            self.initiate_distance_maintenance()

        if self.SHOW_MINH_FUNCTION:
            # Plot the distance between robots
            self.ax_minh = self.fig.add_subplot(gs[1, 2])
            self.ax_minh.set(xlabel="t [s]", ylabel="h min")


    def update(self, time, robot_est_list, robot_cont_list, data):

        for i in robot_est_list:
            est = robot_est_list[i]
            self.canvas.plot_robot_pos(i, est.pos, est.theta)

            if self.SHOW_LIDAR_DETECTION:
                sensed_pos = robot_est_list[i].obs_pos
                try: # update existing plot
                    self.pl_sens[i].set_data(sensed_pos[:, 0], sensed_pos[:, 1])
                except: # Initiate the first time
                    self.pl_sens[i], = self.ax_2D.plot(sensed_pos[:, 0], sensed_pos[:, 1], '.', color=self.canvas._colorList[i])

            if self.SHOW_ROBOT_GOAL:
                goal_pos = robot_cont_list[i].current_goal
                try: # update existing plot
                    self.pl_goal[i].set_data([goal_pos[0]], [goal_pos[1]])
                except: # Initiate the first time
                    self.pl_goal[i], = self.ax_2D.plot([goal_pos[0]], [goal_pos[1]], '.', color=self.canvas._colorList[i])

            # STORE TO another array for other processes
            


        if self.SHOW_CENTROID_ESTIMATION:
            self.plot_centroid_estimation(robot_est_list)

        if self.SHOW_COMMUNICATION:
            self.plot_bidirectional_communication(robot_est_list)


        # Setup for moving window horizon
        max_idx = data.log.get_lastidx()
        if time < self.time_series_window:
            t_range = (-0.1, self.time_series_window + 0.1)
            min_idx = 0
        else:
            t_range = (time - (self.time_series_window + 0.1), time + 0.1)
            min_idx = max_idx - round(self.time_series_window / self.Ts)


        if self.SHOW_DISTANCE_MAINTENACE:
            self.plot_distance_maintenace(data, min_idx, max_idx)
            # Move the time-series window
            self.ax_dist.set(xlim=t_range)

        if self.SHOW_MINH_FUNCTION:
            self.plot_min_h(data, min_idx, max_idx)
            # Move the time-series window
            self.ax_minh.set(xlim=t_range)

        self.draw(time)

    def draw(self, time):
        self.canvas.plot_time(time)
        plt.pause(0.000001) # The pause is needed to show the plot
        # plt.pause(0.5) # Slower pause to observe the movement


    def plot_centroid_estimation(self, robot_est_list):
        # Update each robot estimated centroid
        for i in robot_est_list:
            est_pos = robot_est_list[i].cent_est
            try: # update existing plot
                self.pl_cent[i].set_data([est_pos[0]], [est_pos[1]])
            except: # initiate the first time
                self.pl_cent[i], = self.ax_2D.plot([est_pos[0]], [est_pos[1]], 'x', color=self.canvas._colorList[i])

        # Calculate & Update the true centroid position
        for f_id in self.form_ids:
            ids = self.form_ids[f_id]
            sum_of_pos, robot_num = np.zeros(3), 0            
            for i in ids:
                sum_of_pos += robot_est_list[i].lahead_pos
                robot_num += 1

            true_cent = sum_of_pos / robot_num
            try: # update existing plot
                self.pl_cent[f'f{f_id}'].set_data([true_cent[0]], [true_cent[1]])
            except: # initiate the first time
                self.pl_cent[f'f{f_id}'], = self.ax_2D.plot([true_cent[0]], [true_cent[1]], '+', color='k')


    def plot_bidirectional_communication(self, robot_est_list):
        # Plot for communication graph
        for f_id in self.form_ids:
            ids = self.form_ids[f_id]
            for i in range(len(ids)):
                for j in range(len(ids)):
                    if (i < j) and (self.form_A[f_id][i,j] > 0.):
                        pos_i = robot_est_list[ids[i]].lahead_pos
                        pos_j = robot_est_list[ids[j]].lahead_pos
                        try: # update existing plot
                            self.pl_comm[f'{ids[i]}_{ids[j]}'].set_data(
                                [pos_i[0], pos_j[0]], [pos_i[1], pos_j[1]])
                        except: # initiate the first time
                            self.pl_comm[f'{ids[i]}_{ids[j]}'], = \
                                self.ax_2D.plot([pos_i[0], pos_j[0]], [pos_i[1], pos_j[1]], 
                                                color='k', linewidth=0.5)


    def initiate_distance_maintenance(self):
        temp = np.array([0])
        for f_id in self.form_A:
            A = self.form_A[f_id]
            # Draw the specified band
            temp = np.concat((temp, np.unique(A)))
            # save only the last one: ASSUME ALL THE SAME FOR NOW
            eps = self.form_tol[f_id]
        # Filter only the unique part
        array_req_dist = np.unique(temp)
        array_req_dist = np.delete(array_req_dist, 0)

        for idx, dist in enumerate(array_req_dist):
            if idx == 0:  # only put 1 label
                self.ax_dist.fill_between([0, self.Tmax], [dist - eps], [dist + eps],
                                            alpha=0.12, color='k', linewidth=0, label='specified distance')
            else:
                self.ax_dist.fill_between([0, self.Tmax], [dist - eps], [dist + eps],
                                            alpha=0.12, color='k', linewidth=0)
        # set y-axis
        self.ax_dist.set(ylim=(min(array_req_dist) - eps - 0.1,
                                max(array_req_dist) + eps + 0.1))
        self.ax_dist.grid(True)


    def plot_distance_maintenace(self, data, min_idx, max_idx):
        # Plot time series distance
        time_data = data.log.get_data_from_label('time')
        for f_id in self.form_ids:
            ids = self.form_ids[f_id]
            for i in range(len(ids)):
                for j in range(len(ids)):
                    if (i < j) and (self.form_A[f_id][i,j] > 0.):
                        dist_data = data.log.get_data_from_label(f'dist_f{f_id}-{ids[i]}_{ids[j]}')
                        try: # update existing plot
                            self.pl_dist[f'{ids[i]}_{ids[j]}'].set_data(
                                time_data[min_idx:max_idx], dist_data[min_idx:max_idx])
                        except: # initiate the first time
                            cnt = len(self.pl_dist)
                            self.pl_dist[f'{ids[i]}_{ids[j]}'], = self.ax_dist.plot(
                                time_data[min_idx:max_idx], dist_data[min_idx:max_idx],
                                color=self.tseries_colorList[cnt],
                                label=f'$i={ids[i]}$, $j={ids[j]}$'
                                )
                            # update legend
                            self.ax_dist.legend(loc=(0.65, 0.18), prop={'size': 6})


    def plot_min_h(self, data, min_idx, max_idx):
        # Plot time series distance
        time_data = data.log.get_data_from_label('time')
        for id in self.list_ID:
            min_h = data.log.get_data_from_label(f'min_h{id}')

            try: # update existing plot
                self.pl_minh[id].set_data(
                    time_data[min_idx:max_idx], min_h[min_idx:max_idx])
            except: # initiate the first time
                self.pl_minh[id], = self.ax_minh.plot(
                    time_data[min_idx:max_idx], min_h[min_idx:max_idx],
                    color=self.canvas._colorList[id],
                    label=f'{id}'
                    )
                # update legend
                self.ax_minh.legend(loc=(0.65, 0.18), prop={'size': 6})
                self.ax_minh.grid(True)

        self.ax_minh.set(ylim=(-0.1, data.max_min_h+0.1))




class dataLogger():
    def __init__(self, max_data_num=10000):  # by default store
        self.__data_len = max_data_num
        self._stored_data = {'time': [None] * max_data_num}
        self.__cur_idx = 0

        # only for plotting purposes
        self.__axes_list = {}

    def time_stamp(self, t):  # should be called last after storing all new data
        self._stored_data['time'][self.__cur_idx] = t
        self.__cur_idx += 1  # for next data filling

    def store_data(self, key, value):
        assert self.__cur_idx < self.__data_len, f"Data log exceeds max data_len: {self.__data_len}"
        if not (key in self._stored_data):  # assign default None array
            self._stored_data[key] = [None] * self.__data_len
        # Fill the array starting this index
        self._stored_data[key][self.__cur_idx] = value

    def store_dictionary(self, dict):
        assert self.__cur_idx < self.__data_len, f"Data log exceeds max data_len: {self.__data_len}"
        for key, value in dict.items():
            self.store_data(key, value)

    def get_all_data(self):
        return self._stored_data, self.__cur_idx

    def get_data_from_label(self, label):  # Return up until recent data
        return self._stored_data[label][:self.__cur_idx]

    def get_lastdata_from_label(self, label):  # Return up until recent data
        return self._stored_data[label][self.__cur_idx - 1]

    def get_lastidx(self): 
        return self.__cur_idx

    @staticmethod
    def save_description(path, setup):
        import os
        os.makedirs(path[:path.rfind("/")])
        with open((file_name := path[:path.rfind(".")] + ".txt"), "a") as f:
            print('Storing the setup to into: ' + file_name, flush=True)
            for attr in dir(setup):
                if not attr.startswith("__"):
                    value = eval(f'setup.{attr}').__str__()
                    value = value.replace("\n", " ")
                    f.write(f"{attr:<35}: {value}\n")
            print("Done.")

    def save_to_pkl(self, path):
        import pickle
        print('Storing the data to into: ' + path, flush=True)
        with open(path, 'wb') as f:
            pickle.dump(dict(stored_data=self._stored_data, last_idx=self.__cur_idx - 1), f)
        print('Done.')

    # Plot logged data
    def plot_time_series_batch(self, ax, pre_string):
        # initialize plot to store the plot pointer
        dict_data = {'ax': ax, 'pl': {}}
        # plot all key with matching pre_string
        matches = [key for key in self._stored_data if key.startswith(pre_string)]
        for key in matches:
            dict_data['pl'][key], = dict_data['ax'].plot(0, 0, label=key[len(pre_string):])
        # set grid and legend
        dict_data['ax'].grid(True)
        dict_data['ax'].set(xlabel="t [s]", ylabel=pre_string)
        # store data for update later
        self.__axes_list[pre_string] = dict_data

    def update_time_series_batch(self, pre_string, data_minmax=None):
        # data_minmax should be a tuple with 2 inputs
        if pre_string in self.__axes_list:
            dict_data = self.__axes_list[pre_string]
            # compute the time data
            min_idx, max_idx = 0, self.__cur_idx
            if data_minmax is not None:
                min_idx, max_idx = data_minmax[0], data_minmax[1]
            time_data = self._stored_data['time'][min_idx:max_idx]
            # check all matching keystring
            matches = [key for key in self._stored_data if key.startswith(pre_string)]
            is_new_plot_added = False
            data_min, data_max = 0., 0.
            for key in matches:
                if key in dict_data['pl']:
                    key_data = self._stored_data[key][min_idx:max_idx]
                    dict_data['pl'][key].set_data(time_data, key_data)
                else:  # new data, make new plot
                    key_data = self._stored_data[key][min_idx:max_idx]
                    dict_data['pl'][key], = dict_data['ax'].plot(time_data, key_data,
                                                                 label=key.strip(pre_string))
                    is_new_plot_added = True
                # update min max for plotting
                data_min = min(data_min, min(key_data))
                data_max = max(data_max, max(key_data))
            # adjust time window
            dict_data['ax'].set(xlim=(time_data[0] - 0.1, time_data[-1] + 0.1),
                                ylim=(data_min - 0.1, data_max + 0.1))
            dict_data['ax'].legend(loc='best', prop={'size': 6})
            # update dictionary if needed
            if is_new_plot_added:
                self.__axes_list[pre_string] = dict_data



class ProcessDataLog():
    def __init__(self, param_dict, scenario_dict):
        # Initialize logger
        self.list_ID = scenario_dict.list_robot_ID
        self.log = dataLogger(round(param_dict.Tmax/param_dict.Ts))

        # Save list of robots within each formation
        self.form_ids, self.form_A, self.form_tol = {}, {}, {}
        for f_id in scenario_dict.form_param:
            self.form_ids[f_id] = scenario_dict.form_param[f_id]['ids']
            self.form_A[f_id] = scenario_dict.form_param[f_id]['A']

        self.max_min_h = 0.

    def store_datas(self, time, robot_est_list, robot_cont_list):
        for id in self.list_ID:
            # Log information into logger
            self.log.store_data(f'lahead{id}', robot_est_list[id].lahead_pos)

        # Compute and store distance between robots
        for f_id in self.form_A:
            ids = self.form_ids[f_id]
            for i in range(len(ids)):
                for j in range(len(ids)):
                    if (i < j) and (self.form_A[f_id][i,j] > 0.):
                        pos_i = robot_est_list[ids[i]].lahead_pos
                        pos_j = robot_est_list[ids[j]].lahead_pos
                        dist = np.linalg.norm(pos_i - pos_j)
                        self.log.store_data(f'dist_f{f_id}-{ids[i]}_{ids[j]}', dist)

        # Stamp log
        self.log.time_stamp(time)

    
