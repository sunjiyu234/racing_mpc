from sklearn.metrics import r2_score
from bayes_opt import BayesianOptimization
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events
from bayes_opt import UtilityFunction
from scipy import linalg, signal, interpolate
import numpy as np
from typing import List
# Plot Settings
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg') 
import os

vehicle_name = 'dSPACE vehicle'
vehicle_config = {
    'dSPACE vehicle': {'m1':8870, 'l1':3.9835, 'l_imu':-0.234, 'a':2.0, 'miu':0.85, 'r_wheel': 1.0},
}


################# UTILS ###################################
def deg2rad(angle_deg):
    return angle_deg*np.pi/180

def filter_data(data, wc, Ts):
        # Butterworth filter finds for maxmimum flatness. For example, 1/sqrt(2)
        # of damping ratio for a second-order section, such that the critical
        # freq of the SOS is also the cutoff freq, and passband freq.
		sos = signal.butter(N=4, Wn=wc, btype='low', output='sos', fs=1/Ts)
		return signal.sosfiltfilt(sos, data).tolist()

# def interp_func(x: List[float], t: List[float], t_new: List[float]) -> List[float]:
#     # x-axis t must be monotonously and non-duplicate
#     _, unique_inx = np.unique(t, return_index=True)
#     f = interpolate.interp1d(np.array(t)[unique_inx], np.array(x)[unique_inx])
#     x_new = f(t_new)
#     return x_new.tolist()

def calcTime(num):
    # transfer xxxx seconds as H:M:S format
    a = num // 60
    second = num - a * 60
    hour = a // 60
    minute = a - hour * 60
    return str(hour)+':'+str(minute)+':'+str(second)
###########################################################

class LateralExplictModel:
    def __init__(self, iwheel, f1, f2, k1, h):
        self.iwheel = iwheel
        self.f1 = f1
        self.f2 = f2
        self.k1 = k1
        self.h = h
	
    def step(self, x, delta, torque, dt):

        m1 = vehicle_config[vehicle_name]['m1']
        g = 9.8
        l1 = vehicle_config[vehicle_name]['l1']
        l_imu = vehicle_config[vehicle_name]['l_imu']
        a1 = vehicle_config[vehicle_name]['a']
        miu = vehicle_config[vehicle_name]['miu']
        r_wheel = vehicle_config[vehicle_name]['r_wheel']
        g = 9.81

		# model parameters
        i_wheel = self.iwheel
        f1 = self.f1
        f2 = self.f2
        k1 = self.k1
        h = self.h
        b1 = l1 - a1
        lp = a1 - l_imu
        Iz = m1 * k1**2

        def step_by_linear_model_with_ZOH_method():
            M = [[m1,   		0], \
				[0,        Iz]]
            M = np.asmatrix(M)

            Lp = [[1, -lp], \
				[0,   1]]
            Lp = np.asmatrix(Lp)

            A1 = [[-(C1 + C2)/vx,  -(a1*C1 - b1*C2 + m1*vx**2)/vx], \
				[-(a1*C1 - b1*C2)/vx, -(a1**2*C1 + b1**2*C2)/vx]]
            A1 = np.asmatrix(A1)

            B1 = [[C1], [C1*a1]]
            B1 = np.asmatrix(B1)

            E = [[-g], [0]]

            Ac = Lp.I * M.I * A1 * Lp
            Bc = Lp.I * M.I * B1  

            Ad = linalg.expm(np.dot(dt, Ac))
            Bd = Ac.I * (Ad - np.eye(2)) * Bc
            Ed = Ac.I * (Ad - np.eye(2)) * E

            x_ = np.matmul(Ad, [[x[0]], [x[1]]]) + np.matmul(Bd, [[u + self.tire_offset/self.ksteer]]) + np.matmul(Ed, [[np.sin(d)]]) 
            return [x_[0,0], x_[1,0]]

        def step_by_nonlinear_model_with_Euler_method():
            v = x[0]
            beta = x[1]
            omega = x[2]
            wheel_speed_f = x[3]
            wheel_speed_r = x[4]
            accel_x = x[5]
            
            # tire model
            if wheel_speed_f * r_wheel > v * np.cos(beta):
                kf = (wheel_speed_f * r_wheel - v * np.cos(beta)) / (wheel_speed_f * r_wheel)
            else:
                kf = (wheel_speed_f * r_wheel - v * np.cos(beta)) / (v * np.cos(beta))

            if wheel_speed_r * r_wheel > v * np.cos(beta):
                kr = (wheel_speed_r * r_wheel - v * np.cos(beta)) / (wheel_speed_r * r_wheel)
            else:
                kr = (wheel_speed_r * r_wheel - v * np.cos(beta)) / (v * np.cos(beta))

            Fzf = (m1 * g * b1 - m1 * accel_x * h) / l1
            Fzr = (m1 * g * a1 + m1 * accel_x * h) / l1

            sigma_slf = np.arctan(3.0 * miu * Fzf / f1)
            sigma_slr = np.arctan(3.0 * miu * Fzr / f2)

            alpha_f = np.arctan((v * np.sin(beta) + a1 * omega) / (v * np.cos(beta))) - delta
            alpha_r = np.arctan((v * np.sin(beta) - b1 * omega) / (v * np.cos(beta)))
            
            sigma_f = pow(np.tan(alpha_f) * np.tan(alpha_f) + kf * kf, 0.5)
            sigma_r = pow(np.tan(alpha_r) * np.tan(alpha_r) + kr * kr, 0.5)

            if sigma_f < sigma_slf:
                Ftotal_f = f1 * sigma_f - f1 * f1 * sigma_f * sigma_f / 3.0 / miu / Fzf + f1 * f1 * f1 * sigma_f * sigma_f * sigma_f / 27.0 / (miu * miu * Fzf *Fzf)
            else:
                Ftotal_f = miu * Fzf
            if sigma_r < sigma_slr:
                Ftotal_r = f2 * sigma_r - f2 * f2 * sigma_r * sigma_r / 3.0 / miu / Fzr + f2 * f2 * f2 * sigma_r * sigma_r * sigma_r / 27.0 / (miu * miu * Fzr *Fzr)
            else:
                Ftotal_r = miu * Fzr
            
            Fxf = Ftotal_f * kf / sigma_f
            Fyf = Ftotal_f * (-np.tan(alpha_f)) / sigma_f

            Fxr = Ftotal_r * kr / sigma_r
            Fyr = Ftotal_r * (-np.tan(alpha_r)) / sigma_r

            dot_v = (Fxf * np.cos(delta - beta) - Fyf * np.sin(delta - beta) + Fxf * np.cos(beta) + Fyf * np.sin(beta)) / m1
            dot_beta = (Fxf * np.sin(delta - beta) + Fyf * np.cos(delta - beta) - Fxr * np.sin(beta) + Fyr * np.cos(beta)) / (m1 * v) - omega
            dot_omega = (a1 * (Fxf * np.sin(delta) + Fyf * np.cos(delta)) - b1 * Fyr) / Iz
            if torque > 0:
                dot_wheel_speed_f = (0.0 - Fxf * r_wheel) / i_wheel
                dot_wheel_speed_r = (torque - Fxr * r_wheel) / i_wheel
            else:
                dot_wheel_speed_f = (torque/2.0 - Fxf * r_wheel) / i_wheel
                dot_wheel_speed_r = (torque/2.0 - Fxr * r_wheel) / i_wheel
            
            dot_accelx = (Fxf * np.cos(delta) - Fyf * np.sin(delta) + Fxr) / m1

            v_ = v + dt * dot_v
            beta_ = beta + dt * dot_beta
            omega_ = omega + dt * dot_omega
            wheel_speed_f_ = wheel_speed_f + dt * dot_wheel_speed_f
            wheel_speed_r_ = wheel_speed_r + dt * dot_wheel_speed_r
            accel_x_ = accel_x + dt * dot_accelx
            return [v_, beta_, omega_, wheel_speed_f_, wheel_speed_r_, accel_x_]

        return step_by_nonlinear_model_with_Euler_method()
    
    def simulate(self, x_init, delta_traj, torque_traj, dt):
        y_traj = []

        x = x_init
        for delta, torque in zip(delta_traj, torque_traj):
            y_traj.append(x)
            x_ = self.step(x, delta, torque, dt)
            x = x_
        return y_traj

    def calc_fit_score(self, data, sim):
        return r2_score(data, sim) * 100

    def calc_model_error(self, data_traj, sim_traj):
        model_error = np.array(data_traj) - np.array(sim_traj)
        return model_error.tolist()
    
def checkDataExcitation(lat_data_lst):
        v, steering, beta = [], [], []
        for lat_data in lat_data_lst:
            v += lat_data['v']
            steering += lat_data['delta']
            beta += lat_data['beta']

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        hist, xedges, yedges = np.histogram2d(
            np.array(v)*3.6,
            np.array(steering),
            bins=20, range=[[0, 120], [-5, 5]])

        # Construct arrays for the anchor positions of the 20*20 bars.
        xpos, ypos = np.meshgrid(xedges[:-1] + 0.25, yedges[:-1] + 0.25, indexing="ij")
        xpos = xpos.ravel()
        ypos = ypos.ravel()
        zpos = 0

        # Construct arrays with the dimensions for the 20*20 bars.
        dx = dy = 0.5 * np.ones_like(zpos)
        dz = hist.ravel()

        ax.bar3d(xpos, ypos, zpos, dx, dy, dz, zsort='average')

        a_vx_beta = fig.add_subplot(projection= '3d')
        hist2, xedges2, yedges2 = np.histogram2d(np.array(v)*3.6,
                                                 np.array(beta),
                                                 bins=20, range= [[0, 120], [-1, 1]])
        xpos2, ypos2 = np.meshgrid(xedges2[:-1] + 0.25, yedges2[:-1] + 0.25, indexing="ij")
        xpos2 = xpos2.ravel()
        ypos2 = ypos2.ravel()
        zpos2 = 0

        # Construct arrays with the dimensions for the 20*20 bars.
        dx2 = dy2 = 0.5 * np.ones_like(zpos2)
        dz2 = hist2.ravel()

        a_vx_beta.bar3d(xpos2, ypos2, zpos2, dx2, dy2, dz2, zsort='average')

        plt.show()



def evaluate_model_accuracy(lat_model, test_lat_data_lst, sampling_rate):
    sim_v, data_v, sim_beta, data_beta, sim_yaw_rate, data_yaw_rate, sim_wheel_speed_f, data_wheel_speed_f, sim_wheel_speed_r, data_wheel_speed_r, sim_accel_x, data_accel_x, time = [], [], [], [], [], [], [], [], [], [], [], [], []
    for lat_data in test_lat_data_lst:
        beta_lst = lat_data['beta']
        omega_lst = lat_data['omega']
        delta_lst = lat_data['delta']
        wheelspeed_f_lst = lat_data['wheel_speed_f']
        wheelspeed_r_lst = lat_data['wheel_speed_r']
        accelx_lst = lat_data['ax']
        torque_lst = lat_data['torque']
        v_lst = lat_data['v']
    
        if (len(v_lst) > 0):
            x_init = [v_lst[0], beta_lst[0], omega_lst[0], wheelspeed_f_lst[0], wheelspeed_r_lst[0], accelx_lst[0]]
            sim_output = lat_model.simulate(x_init, delta_traj = delta_lst, torque_traj = torque_lst, dt = sampling_rate)
            sim_v += np.array(sim_output)[:,0].tolist()
            data_v += v_lst
            sim_beta += np.array(sim_output)[:,1].tolist()
            data_beta += beta_lst
            sim_yaw_rate += np.array(sim_output)[:,2].tolist()
            data_yaw_rate += omega_lst
            sim_wheel_speed_f += np.array(sim_output)[:,3].tolist()
            data_wheel_speed_f += wheelspeed_f_lst
            sim_wheel_speed_r += np.array(sim_output)[:,4].tolist()
            data_wheel_speed_r += wheelspeed_r_lst	
            sim_accel_x += np.array(sim_output)[:,5].tolist()
            data_accel_x += accelx_lst			
            time += lat_data['time']

    yaw_rate_error = lat_model.calc_model_error(data_yaw_rate, sim_yaw_rate)
    v_error = lat_model.calc_model_error(data_v, sim_v)
    beta_error = lat_model.calc_model_error(data_beta, sim_beta)
    wheelspeed_f_error = lat_model.calc_model_error(data_wheel_speed_f, sim_wheel_speed_f)
    wheelspeed_r_error = lat_model.calc_model_error(data_wheel_speed_r, sim_wheel_speed_r)
    accelx_error = lat_model.calc_model_error(data_accel_x, sim_accel_x)

    yaw_rate_fit_score = lat_model.calc_fit_score(data_yaw_rate, sim_yaw_rate)
    v_fit_score = lat_model.calc_fit_score(data_v, sim_v)
    beta_fit_score = lat_model.calc_fit_score(data_beta, sim_beta)
    wheelspeed_f_fit_score = lat_model.calc_fit_score(data_wheel_speed_f, sim_wheel_speed_f)
    wheelspeed_r_fit_score = lat_model.calc_fit_score(data_wheel_speed_r, sim_wheel_speed_r)
    accelx_fit_score = lat_model.calc_fit_score(data_accel_x, sim_accel_x)

    fig = plt.figure(figsize=(24, 14), dpi = 80)
    ax1 = plt.subplot2grid((3, 3), (0, 0), rowspan=1, colspan=2)
    ax2 = plt.subplot2grid((3, 3), (0, 2))
    ax3 = plt.subplot2grid((3, 3), (1, 0), rowspan=1, colspan=2)
    ax4 = plt.subplot2grid((3, 3), (1, 2))
    ax5 = plt.subplot2grid((3, 3), (2, 0), rowspan=1, colspan=2)
    ax6 = plt.subplot2grid((3, 3), (2, 2))

    ax1.plot(time, data_yaw_rate, color = 'tab:blue', alpha = 0.6, label = 'data', linewidth = 0.5)
    ax1.plot(time, sim_yaw_rate, color = 'tab:red', label = 'sim', linewidth = 2)
    ax3.plot(time, data_v, color = 'tab:blue', alpha = 0.6, label = 'data', linewidth = 0.5)
    ax3.plot(time, sim_v, color = 'tab:red', label = 'sim', linewidth = 2)
    ax5.plot(time, data_beta, color = 'tab:blue', alpha = 0.6, label = 'data', linewidth = 0.5)
    ax5.plot(time, sim_beta, color = 'tab:red', label = 'sim', linewidth = 2)

    ax1.set_xlabel('time (s)', labelpad = 10)
    ax1.set_ylabel('yaw rate (rad/s)', labelpad = 10)
    ax1.set_title('Fit Score: ' + str(round(yaw_rate_fit_score, 2)))
    ax1.grid('both', alpha = .2)
    ax1.legend()

    ax3.set_xlabel('time (s)', labelpad = 10)
    ax3.set_ylabel('velocity (m/s)', labelpad = 10)
    ax3.set_title('Fit Score: ' + str(round(v_fit_score, 2)))
    ax3.grid('both', alpha = .2)
    ax3.legend()

    ax5.set_xlabel('time (s)', labelpad = 10)
    ax5.set_ylabel('velocity (m/s)', labelpad = 10)
    ax5.set_title('Fit Score: ' + str(round(beta_fit_score, 2)))
    ax5.grid('both', alpha = .2)
    ax5.legend()

    err_mean = round(np.mean(yaw_rate_error), 4)
    err_std = round(np.std(yaw_rate_error), 4)
    ax2.hist(yaw_rate_error, bins = (np.arange(-0.02, 0.022, 0.002)).tolist(), color = 'tab:blue')
    ax2.set_xlabel('model yaw rate error (rad/s)', labelpad = 10)
    ax2.grid('both', alpha = .2)
    ax2.set_title('Mean: ' + str(err_mean) + ' STD: ' + str(err_std))

    err_mean = round(np.mean(v_error), 4)
    err_std = round(np.std(v_error), 4)
    ax4.hist(v_error, bins = (np.arange(-5.0, 5.0, 0.2)).tolist(), color = 'tab:blue')
    ax4.set_xlabel('model vel error (m/s)', labelpad = 10)
    ax4.grid('both', alpha = .2)
    ax4.set_title('Mean: ' + str(err_mean) + ' STD: ' + str(err_std))

    err_mean = round(np.mean(beta_error), 4)
    err_std = round(np.std(beta_error), 4)
    ax6.hist(beta_error, bins = (np.arange(-0.2, 0.2, 0.01)).tolist(), color = 'tab:blue')
    ax6.set_xlabel('model beta error (rad)', labelpad = 10)
    ax6.grid('both', alpha = .2)
    ax6.set_title('Mean: ' + str(err_mean) + ' STD: ' + str(err_std))

    fig2 = plt.figure(figsize=(24, 14), dpi = 80)
    ay1 = plt.subplot2grid((3, 3), (0, 0), rowspan=1, colspan=2)
    ay2 = plt.subplot2grid((3, 3), (0, 2))
    ay3 = plt.subplot2grid((3, 3), (1, 0), rowspan=1, colspan=2)
    ay4 = plt.subplot2grid((3, 3), (1, 2))
    ay5 = plt.subplot2grid((3, 3), (2, 0), rowspan=1, colspan=2)
    ay6 = plt.subplot2grid((3, 3), (2, 2))

    ay1.plot(time, data_wheel_speed_f, color = 'tab:blue', alpha = 0.6, label = 'data', linewidth = 0.5)
    ay1.plot(time, sim_wheel_speed_f, color = 'tab:red', label = 'sim', linewidth = 2)
    ay3.plot(time, data_wheel_speed_r, color = 'tab:blue', alpha = 0.6, label = 'data', linewidth = 0.5)
    ay3.plot(time, sim_wheel_speed_r, color = 'tab:red', label = 'sim', linewidth = 2)
    ay5.plot(time, data_accel_x, color = 'tab:blue', alpha = 0.6, label = 'data', linewidth = 0.5)
    ay5.plot(time, sim_accel_x, color = 'tab:red', label = 'sim', linewidth = 2)

    ay1.set_xlabel('time (s)', labelpad = 10)
    ay1.set_ylabel('wheel speed f (rad/s)', labelpad = 10)
    ay1.set_title('Fit Score: ' + str(round(wheelspeed_f_fit_score, 2)))
    ay1.grid('both', alpha = .2)
    ay1.legend()

    ay3.set_xlabel('time (s)', labelpad = 10)
    ay3.set_ylabel('wheel speed r (rad/s)', labelpad = 10)
    ay3.set_title('Fit Score: ' + str(round(wheelspeed_r_fit_score, 2)))
    ay3.grid('both', alpha = .2)
    ay3.legend()

    ay5.set_xlabel('time (s)', labelpad = 10)
    ay5.set_ylabel('accel x (m/s^2)', labelpad = 10)
    ay5.set_title('Fit Score: ' + str(round(accelx_fit_score, 2)))
    ay5.grid('both', alpha = .2)
    ay5.legend()

    err_mean = round(np.mean(wheelspeed_f_error), 4)
    err_std = round(np.std(wheelspeed_f_error), 4)
    ay2.hist(wheelspeed_f_error, bins = (np.arange(-0.2, 0.2, 0.02)).tolist(), color = 'tab:blue')
    ay2.set_xlabel('model wheel speed f error (rad/s)', labelpad = 10)
    ay2.grid('both', alpha = .2)
    ay2.set_title('Mean: ' + str(err_mean) + ' STD: ' + str(err_std))

    err_mean = round(np.mean(wheelspeed_r_error), 4)
    err_std = round(np.std(wheelspeed_r_error), 4)
    ay4.hist(wheelspeed_r_error, bins = (np.arange(-0.2, 0.2, 0.02)).tolist(), color = 'tab:blue')
    ay4.set_xlabel('model wheel speed r error (rad/s)', labelpad = 10)
    ay4.grid('both', alpha = .2)
    ay4.set_title('Mean: ' + str(err_mean) + ' STD: ' + str(err_std))

    err_mean = round(np.mean(accelx_error), 4)
    err_std = round(np.std(accelx_error), 4)
    ay6.hist(accelx_error, bins = (np.arange(-0.5, 0.5, 0.02)).tolist(), color = 'tab:blue')
    ay6.set_xlabel('model accelx error (rad)', labelpad = 10)
    ay6.grid('both', alpha = .2)
    ay6.set_title('Mean: ' + str(err_mean) + ' STD: ' + str(err_std))

    plt.show()


if __name__ == "__main__":
    sampling_rate = 0.02
    
    train_lat_data_lst = [{'beta': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'omega': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'delta': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'wheel_speed_f': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'wheel_speed_r': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'ax': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'torque': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'v': [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
                            'time': [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18]},
                            {'beta': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'omega': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'delta': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'wheel_speed_f': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'wheel_speed_r': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'ax': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'torque': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'v': [11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0],
                            'time': [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18]}
                            ]
    checkDataExcitation(train_lat_data_lst)
    print('Finish Training Data Prepare and Check')

    #### Bayes Optimization ###########################################
    def func_params2cost(Iwheel, f1, f2, k1, h):
        #NOTE: func params can not be data or algorithm, can only be params to be identified
        lat_model = LateralExplictModel(Iwheel, f1, f2, k1, h)

        cost = 0
        weight_v = 1
        weight_beta = 1
        weight_omega = 1
        weight_w_f = 1
        weight_w_r = 1
        weight_accelx = 1
        for lat_data in train_lat_data_lst:
            beta_lst = lat_data['beta']
            omega_lst = lat_data['omega']
            delta_lst = lat_data['delta']
            wheelspeed_f_lst = lat_data['wheel_speed_f']
            wheelspeed_r_lst = lat_data['wheel_speed_r']
            accelx_lst = lat_data['ax']
            torque_lst = lat_data['torque']
            v_lst = lat_data['v']

            init_state = [v_lst[0], beta_lst[0], omega_lst[0], wheelspeed_f_lst[0], wheelspeed_r_lst[0], accelx_lst[0]]
            sim_output = lat_model.simulate(
                init_state,
                delta_lst,
                torque_lst,
                sampling_rate,
            )
            v_sim = np.array(sim_output)[:,0].tolist()
            beta_sim = np.array(sim_output)[:,1].tolist()
            omega_sim = np.array(sim_output)[:,2].tolist()
            wheelspeed_f_sim = np.array(sim_output)[:,3].tolist()
            wheelspeed_r_sim = np.array(sim_output)[:,4].tolist()
            accelx_sim = np.array(sim_output)[:,5].tolist()

            v_error_norm = np.linalg.norm(np.array(v_sim) - np.array(v_lst))
            beta_error_norm = np.linalg.norm(np.array(beta_sim) - np.array(beta_lst))
            wheelspeed_f_error_norm = np.linalg.norm(np.array(wheelspeed_f_sim) - np.array(wheelspeed_f_lst))
            wheelspeed_r_error_norm = np.linalg.norm(np.array(wheelspeed_r_sim) - np.array(wheelspeed_r_lst))
            omega_error_norm = np.linalg.norm(np.array(omega_sim) - np.array(omega_lst))
            accelx_norm = np.linalg.norm(np.array(accelx_sim) - np.array(accelx_lst))
            cost -= v_error_norm * weight_v + beta_error_norm * weight_beta + wheelspeed_f_error_norm * weight_w_f + weight_w_r * wheelspeed_r_error_norm + omega_error_norm*weight_omega + weight_accelx * accelx_norm  #NOTE: bayes_opt can only find maximum
        
        return cost

    #NOTE: Bounds could be changed during optimzatin, using opt.set_bounds() and then opt.maximize() again
    pbounds = {
        'Iwheel': (1.0, 1.5),
        'f1': (5.1, 5.7),
        'f2': (24.0, 28.0),
        'k1': (2.5, 3.5),
        'h':(0.8, 1.2),
    }
    opt = BayesianOptimization(
        f = func_params2cost,
        pbounds = pbounds,
        random_state = 53
    )

    #NOTE: Observer to save and load progress
    result_path = '/home/sun234/racing_work/src/offline_bayesian_optimization/'
    if not os.path.exists(result_path):
        os.makedirs(result_path)
    logger = JSONLogger(path=result_path + 'opt_details.json')
    opt.subscribe(Events.OPTIMIZATION_STEP, logger)

    ###### dSPACE vehicle
    #opt.probe(params=
    #  {'a1': 1.1776168416169248, 'f1': 5.641414394815659, 'f2': 26.328454650667563, 'k1': 3.061283864705716, 'ksteer': 1.0626948840487278}, lazy=True)
    opt.probe(params=
      {'Iwheel': 1.2, 'f1': 5.5, 'f2': 24.68, 'k1': 3.0, 'h': 1.0}, lazy=True)
    acquisition_func = UtilityFunction(kind='ucb', kappa=10.0)  # kappa=10 prefers exploration, kappa=0.1 prefers exploitation
    print("here")
    opt.maximize(init_points=2, n_iter=100, acquisition_function=acquisition_func)
    print(opt.max)

    #### Model Validation ###############################################
    lat_model_identified = LateralExplictModel(
        opt.max['params']['Iwheel'],
        opt.max['params']['f1'],
        opt.max['params']['f2'],
        opt.max['params']['k1'],
        opt.max['params']['h'],
    )
    test_lat_data_lst = [{'beta': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'omega': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'delta': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'wheel_speed_f': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'wheel_speed_r': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'ax': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'torque': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'v': [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
                            'time': [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18]},
                            {'beta': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'omega': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'delta': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'wheel_speed_f': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'wheel_speed_r': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'ax': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'torque': [0.0, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03, 0.01, 0.03, 0.03],
                            'v': [11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0, 11.0],
                            'time': [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18]}
                            ]
    checkDataExcitation(test_lat_data_lst)
    evaluate_model_accuracy(lat_model_identified, test_lat_data_lst, sampling_rate)
