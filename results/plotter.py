import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
 
# Specify the *.csv dataset filename.
# These *.csv files are converted from *.bag files using the rosbag_to_csv package
# by Atsushi Sakai: https://github.com/AtsushiSakai/rosbag_to_csv
# 1: groundtruth_pose       : Ground truth dataset for the pose (position & orientation) states.
#                             This is from Wuest et al's original dataset (lissajous_trajectory.bag)
# 2: groundtruth_velocity   : Ground truth dataset for the translational & angular velocity states.
#                             (estimator.launch run on lissajous_trajectory.bag)
#                             OR 
#                             just use geomInertiaEstimator_UKF_results.bag
# 3: estimated_states       : Estimated states of the proposed VDM-aided INS 
#                             (vdm_ins.launch run on either lissajous_trajectory.bag OR geomInertiaEstimator_UKF_results.bag). 

groundtruth_pose_filename = '2022-07-12-22-54-12-quadrotor-pose.csv'
groundtruth_velocity_filename = 'ukfParamEstimation_2022-06-28-00-39-30-quadrotor-geom_inertia_estimator-param_estimates.csv'
estimated_states_filename = 'lambda_3e-6_2022-07-18-09-58-07-quadrotor-geom_inertia_estimator-param_estimates.csv'

# Convert orientation representation from quaternion (qx,qy,qz,qw) to Euler angles (roll, pitch, yaw)
def quaternion_to_euler(x, y, z, w):
    r1 = 2.0*(w*x + y*z)
    r2 = 1.0 - 2.0*(x*x + y*y)
    roll = 180/math.pi * math.atan2(r1, r2)
    
    p1 = math.sqrt(1.0 + 2.0*(w*y - x*z))
    p2 = math.sqrt(1.0 - 2.0*(w*y - x*z))
    pitch = 180/math.pi * (-math.pi/2 + 2*math.atan2(p1, p2))
 
    y1 = 2.0*(w*z + x*y)
    y2 = 1.0 - 2.0*(y*y + z*z)
    yaw = 180/math.pi*math.atan2(y1, y2)
 
    return roll, pitch, yaw # (degrees)

def quaternion_to_euler_array(x, y, z, w):
    roll    = np.array([])
    pitch   = np.array([])
    yaw     = np.array([])

    for idx,_ in enumerate(x):
        _roll, _pitch, _yaw = quaternion_to_euler(x[idx], y[idx], z[idx], w[idx])
        roll    = np.append(roll, _roll)
        pitch   = np.append(pitch, _pitch)
        yaw     = np.append(yaw, _yaw)

    return roll, pitch, yaw

# Ground truth pose data extraction
gt_pose_df          = pd.read_csv(groundtruth_pose_filename)
unixTime            = pd.to_datetime(gt_pose_df['time'])
gt_pose_df['time']  = unixTime - unixTime[0]
gt_pose_time        = gt_pose_df['time'].dt.total_seconds()
gt_pose_time        = gt_pose_time.to_numpy()

# The y & z axes pose are inverted in Wuest et al's groundtruth dataset
gt_r_x = gt_pose_df['.pose.pose.position.x'].to_numpy()
gt_r_y = -gt_pose_df['.pose.pose.position.y'].to_numpy()
gt_r_z = -gt_pose_df['.pose.pose.position.z'].to_numpy()
gt_q_x = gt_pose_df['.pose.pose.orientation.x'].to_numpy()
gt_q_y = -gt_pose_df['.pose.pose.orientation.y'].to_numpy()
gt_q_z = -gt_pose_df['.pose.pose.orientation.z'].to_numpy()
gt_q_w = gt_pose_df['.pose.pose.orientation.w'] .to_numpy()


# Ground truth velocity data extraction
gt_vel_df           = pd.read_csv(groundtruth_velocity_filename)
unixTime            = pd.to_datetime(gt_vel_df['time'])
gt_vel_df['time']   = unixTime - unixTime[0]
gt_vel_time         = gt_vel_df['time'].dt.total_seconds()
gt_vel_time         = gt_vel_time.to_numpy()

gt_v_x = gt_vel_df['.v.x'].to_numpy()
gt_v_y = gt_vel_df['.v.y'].to_numpy()
gt_v_z = gt_vel_df['.v.z'].to_numpy()
gt_w_x = gt_vel_df['.Omega.x'].to_numpy()
gt_w_y = gt_vel_df['.Omega.y'].to_numpy()
gt_w_z = gt_vel_df['.Omega.z'].to_numpy()


# Estimated states data extraction
est_df          = pd.read_csv(estimated_states_filename)
unixTime        = pd.to_datetime(est_df['time'])
est_df['time']  = unixTime - unixTime[0]
tdelta_est_time = (unixTime - unixTime[0]).dt.total_seconds()
est_time        = tdelta_est_time.to_numpy()
est_r_x         = est_df['.r.x'].to_numpy()
est_r_y         = est_df['.r.y'].to_numpy()
est_r_z         = est_df['.r.z'].to_numpy()
est_v_x         = est_df['.v.x'].to_numpy()
est_v_y         = est_df['.v.y'].to_numpy()
est_v_z         = est_df['.v.z'].to_numpy()
est_q_x         = est_df['.q.x'].to_numpy()
est_q_y         = est_df['.q.y'].to_numpy()
est_q_z         = est_df['.q.z'].to_numpy()
est_q_w         = est_df['.q.w'].to_numpy()
est_w_x         = est_df['.Omega.x'].to_numpy()
est_w_y         = est_df['.Omega.y'].to_numpy()
est_w_z         = est_df['.Omega.z'].to_numpy()
est_cov         = est_df['.covariance'].to_numpy()


# Resample estimated states and groundtruth velocity according to groundtruth pose time 
# (the slowest sampling between these three datasets)

print("len(gt_pose_time): ", len(gt_pose_time))
print("len(gt_vel_time): ", len(gt_vel_time))
print("len(est_time): ", len(est_time))

# Iterate through gt_pose_time and find the closest time index for est_time & gt_vel_time
est_time_idx = []
gt_vel_time_idx = []
for gt_idx,gt_val in enumerate(gt_pose_time):
    est_diff_array = np.absolute(gt_val - est_time)
    est_idx = est_diff_array.argmin()
    est_time_idx.append(est_idx)

    gt_vel_diff_array = np.absolute(gt_val - gt_vel_time)
    gt_vel_idx = gt_vel_diff_array.argmin()
    gt_vel_time_idx.append(gt_vel_idx)
    
est_time_resampled = np.take(est_time, est_time_idx)
est_r_x_resampled = np.take(est_r_x, est_time_idx)
est_r_y_resampled = np.take(est_r_y, est_time_idx)
est_r_z_resampled = np.take(est_r_z, est_time_idx)
est_v_x_resampled = np.take(est_v_x, est_time_idx)
est_v_y_resampled = np.take(est_v_y, est_time_idx)
est_v_z_resampled = np.take(est_v_z, est_time_idx)
est_q_x_resampled = np.take(est_q_x, est_time_idx)
est_q_y_resampled = np.take(est_q_y, est_time_idx)
est_q_z_resampled = np.take(est_q_z, est_time_idx)
est_q_w_resampled = np.take(est_q_w, est_time_idx)
est_w_x_resampled = np.take(est_w_x, est_time_idx)
est_w_y_resampled = np.take(est_w_y, est_time_idx)
est_w_z_resampled = np.take(est_w_z, est_time_idx) 

gt_v_x_resampled = np.take(gt_v_x, gt_vel_time_idx)
gt_v_y_resampled = np.take(gt_v_y, gt_vel_time_idx)
gt_v_z_resampled = np.take(gt_v_z, gt_vel_time_idx)
gt_w_x_resampled = np.take(gt_w_x, gt_vel_time_idx)
gt_w_y_resampled = np.take(gt_w_y, gt_vel_time_idx)
gt_w_z_resampled = np.take(gt_w_z, gt_vel_time_idx)

print("len(est_time_resampled): ", len(est_time_resampled))

# Root mean square error (RMSE) between groundtruth and estimation
RMSE_r_x = math.sqrt(np.square(np.subtract(est_r_x_resampled, gt_r_x)).mean())
RMSE_r_y = math.sqrt(np.square(np.subtract(est_r_y_resampled, gt_r_y)).mean())
RMSE_r_z = math.sqrt(np.square(np.subtract(est_r_z_resampled, gt_r_z)).mean())
RMSE_v_x = math.sqrt(np.square(np.subtract(est_v_x_resampled, gt_v_x_resampled)).mean())
RMSE_v_y = math.sqrt(np.square(np.subtract(est_v_y_resampled, gt_v_y_resampled)).mean())
RMSE_v_z = math.sqrt(np.square(np.subtract(est_v_z_resampled, gt_v_z_resampled)).mean())
RMSE_q_x = math.sqrt(np.square(np.subtract(est_q_x_resampled, gt_q_x)).mean())
RMSE_q_y = math.sqrt(np.square(np.subtract(est_q_y_resampled, gt_q_y)).mean())
RMSE_q_z = math.sqrt(np.square(np.subtract(est_q_z_resampled, gt_q_z)).mean())
RMSE_q_w = math.sqrt(np.square(np.subtract(est_q_w_resampled, gt_q_w)).mean())
RMSE_w_x = math.sqrt(np.square(np.subtract(est_w_x_resampled, gt_w_x_resampled)).mean())
RMSE_w_y = math.sqrt(np.square(np.subtract(est_w_y_resampled, gt_w_y_resampled)).mean())
RMSE_w_z = math.sqrt(np.square(np.subtract(est_w_z_resampled, gt_w_z_resampled)).mean()) 

gt_roll, gt_pitch, gt_yaw                                   = quaternion_to_euler_array(gt_q_x, gt_q_y, gt_q_z, gt_q_w)
est_roll, est_pitch, est_yaw                                = quaternion_to_euler_array(est_q_x, est_q_y, est_q_z, est_q_w)
est_roll_resampled, est_pitch_resampled, est_yaw_resampled  = quaternion_to_euler_array(est_q_x_resampled, est_q_y_resampled, est_q_z_resampled, est_q_w_resampled)

RMSE_roll   = math.sqrt(np.square(np.subtract(est_roll_resampled, gt_roll)).mean())
RMSE_pitch  = math.sqrt(np.square(np.subtract(est_pitch_resampled, gt_pitch)).mean())
RMSE_yaw    = math.sqrt(np.square(np.subtract(est_yaw_resampled, gt_yaw)).mean())

print("RMSE_r_x: ", RMSE_r_x)
print("RMSE_r_y: ", RMSE_r_y)
print("RMSE_r_z: ", RMSE_r_z)
print("RMSE_v_x: ", RMSE_v_x)
print("RMSE_v_y: ", RMSE_v_y)
print("RMSE_v_z: ", RMSE_v_z)
print("RMSE_q_x: ", RMSE_q_x)
print("RMSE_q_y: ", RMSE_q_y)
print("RMSE_q_z: ", RMSE_q_z)
print("RMSE_q_w: ", RMSE_q_w)
print("RMSE_w_x: ", RMSE_w_x)
print("RMSE_w_y: ", RMSE_w_y)
print("RMSE_w_z: ", RMSE_w_z)
print("RMSE_roll: ", RMSE_roll)
print("RMSE_pitch: ", RMSE_pitch)
print("RMSE_yaw: ", RMSE_yaw)


# Position Plot
fig_pos, ax_pos = plt.subplots(3,1)
fig_pos.set_size_inches(w=5.5, h=4.5) #h=4.5) #(w=3.5, h=4)
fig_pos.suptitle('Quadrotor Position w.r.t. World Frame')

ax_pos[0].plot(gt_pose_time, gt_r_x, label='Ground truth', color = 'red')
ax_pos[0].plot(est_time, est_r_x, label='Estimated', color = 'blue', linewidth=1)
ax_pos[0].set_ylabel('$x$ pos. ($m$)', fontsize = 7)
ax_pos[0].set_xlim([0, 25])
ax_pos[0].tick_params(axis='x', labelsize = 6)
ax_pos[0].tick_params(axis='y', labelsize = 6)
ax_pos[0].grid()
# ax_pos[0].text(20, -2, 'RMSE: '+'%.4f'%RMSE_r_x, fontsize=9)

ax_pos[1].plot(gt_pose_time, gt_r_y, label='Ground truth', color = 'red')
ax_pos[1].plot(est_time, est_r_y, label='Estimated', color = 'blue', linewidth=1)
ax_pos[1].set_ylabel('$y$ pos. ($m$)', fontsize = 7)
ax_pos[1].set_xlim([0, 25])
ax_pos[1].tick_params(axis='x', labelsize = 6)
ax_pos[1].tick_params(axis='y', labelsize = 6)
ax_pos[1].grid()
# ax_pos[1].text(19.5, 5, 'RMSE: '+'%.4f'%RMSE_r_y, fontsize=9)

ax_pos[2].plot(gt_pose_time, gt_r_z, label='Ground truth', color = 'red')
ax_pos[2].plot(est_time, est_r_z, label='Estimated', color = 'blue', linewidth=1)
ax_pos[2].set_ylabel('$z$ pos. ($m$)', fontsize = 7)
ax_pos[2].set_xlim([0, 25])
ax_pos[2].tick_params(axis='x', labelsize = 6)
ax_pos[2].tick_params(axis='y', labelsize = 6)
ax_pos[2].grid()
ax_pos[2].legend(bbox_to_anchor=(-0.1, -0.5), loc = 'lower left', fontsize = 7, ncol=2)
ax_pos[2].set_xlabel('Time (seconds)', fontsize = 7)
# ax_pos[2].text(20, 1.32, 'RMSE: '+'%.4f'%RMSE_r_z, fontsize=9)


# Quaternion Orientation Plot
fig_quat, ax_quat = plt.subplots(4,1)
fig_quat.set_size_inches(w=5.5, h=5.5) #(w=3.5, h=4)
fig_quat.suptitle('Quadrotor Quaternion Orientation w.r.t. World Frame')

ax_quat[0].plot(gt_pose_time, gt_q_w, label='Ground truth', color = 'red')
ax_quat[0].plot(est_time, est_q_w, label='Estimated', color = 'blue', linewidth=1)
ax_quat[0].set_xlim([0, 25])
ax_quat[0].set_ylabel('$q_w$', fontsize = 9)
ax_quat[0].tick_params(axis='x', labelsize = 6)
ax_quat[0].tick_params(axis='y', labelsize = 6)
ax_quat[0].grid()
# ax_quat[0].text(15, -0.4, 'RMSE: '+'%.4f'%RMSE_q_w, fontsize=9)

ax_quat[1].plot(gt_pose_time, gt_q_x, label='Ground truth', color = 'red')
ax_quat[1].plot(est_time, est_q_x, label='Estimated', color = 'blue', linewidth=1)
ax_quat[1].set_ylabel('$q_x$', fontsize = 9)
ax_quat[1].set_xlim([0, 25])
ax_quat[1].tick_params(axis='x', labelsize = 6)
ax_quat[1].tick_params(axis='y', labelsize = 6)
ax_quat[1].grid()
# ax_quat[1].text(20, -0.4, 'RMSE: '+'%.4f'%RMSE_q_x, fontsize=9)

ax_quat[2].plot(gt_pose_time, gt_q_y, label='Groundtruth', color = 'red')
ax_quat[2].plot(est_time, est_q_y, label='Estimated', color = 'blue', linewidth=1)
ax_quat[2].set_ylabel('$q_y$', fontsize = 9)
ax_quat[2].set_xlim([0, 25])
ax_quat[2].tick_params(axis='x', labelsize = 6)
ax_quat[2].tick_params(axis='y', labelsize = 6)
ax_quat[2].grid()
# ax_quat[2].text(3, -0.25, 'RMSE: '+'%.4f'%RMSE_q_y, fontsize=9)

ax_quat[3].plot(gt_pose_time, gt_q_z, label='Groundtruth', color = 'red')
ax_quat[3].plot(est_time, est_q_z, label='Estimated', color = 'blue', linewidth=1)
ax_quat[3].set_ylabel('$q_z$', fontsize = 9)
ax_quat[3].set_xlim([0, 25])
ax_quat[3].tick_params(axis='x', labelsize = 6)
ax_quat[3].tick_params(axis='y', labelsize = 6)
ax_quat[3].legend(bbox_to_anchor=(-0.1, -0.5), loc = 'lower left', fontsize = 7, ncol=2)
ax_quat[3].set_xlabel('Time (seconds)', fontsize = 7)
ax_quat[3].grid()
# ax_quat[3].text(21, 0.6, 'RMSE: '+'%.4f'%RMSE_q_z, fontsize=9)


# Euler Orientation Plot
fig_rpy, ax_rpy = plt.subplots(3,1)
fig_rpy.set_size_inches(w=5.5, h=4.5) #(w=3.5, h=4)
fig_rpy.suptitle('Quadrotor Euler Orientation w.r.t. World Frame')

ax_rpy[0].plot(gt_pose_time, gt_roll, label='Ground truth', color = 'red')
ax_rpy[0].plot(est_time, est_roll, label='Estimated', color = 'blue', linewidth=1)
ax_rpy[0].set_xlim([0, 25])
ax_rpy[0].set_ylabel('Roll ($^{\circ}$)', fontsize = 7)
ax_rpy[0].tick_params(axis='x', labelsize = 6)
ax_rpy[0].tick_params(axis='y', labelsize = 6)
ax_rpy[0].grid()
# ax_rpy[0].text(15, -60, 'RMSE: '+'%.4f'%RMSE_roll, fontsize=9)

ax_rpy[1].plot(gt_pose_time, gt_pitch, label='Ground truth', color = 'red')
ax_rpy[1].plot(est_time, est_pitch, label='Estimated', color = 'blue', linewidth=1)
ax_rpy[1].set_ylabel('Pitch ($^{\circ}$)', fontsize = 7)
ax_rpy[1].set_xlim([0, 25])
ax_rpy[1].tick_params(axis='x', labelsize = 6)
ax_rpy[1].tick_params(axis='y', labelsize = 6)
ax_rpy[1].grid()
# ax_rpy[1].text(20, 40, 'RMSE: '+'%.4f'%RMSE_pitch, fontsize=9)

ax_rpy[2].plot(gt_pose_time, gt_yaw, label='Groundtruth', color = 'red')
ax_rpy[2].plot(est_time, est_yaw, label='Estimated', color = 'blue', linewidth=1)
ax_rpy[2].set_ylabel('Yaw ($^{\circ}$)', fontsize = 7)
ax_rpy[2].set_xlim([0, 25])
ax_rpy[2].tick_params(axis='x', labelsize = 6)
ax_rpy[2].tick_params(axis='y', labelsize = 6)
ax_rpy[2].legend(bbox_to_anchor=(-0.1, -0.5), loc = 'lower left', fontsize = 7, ncol=2)
ax_rpy[2].set_xlabel('Time (seconds)', fontsize = 7)
ax_rpy[2].grid()
# ax_rpy[2].text(3, -50, 'RMSE: '+'%.4f'%RMSE_yaw, fontsize=9)


# Velocity Plot
fig_vel, ax_vel = plt.subplots(3,1)
fig_vel.set_size_inches(w=5.5, h=4.5) #(w=3.5, h=4)
fig_vel.suptitle('Quadrotor Translational Velocity w.r.t. World Frame')

ax_vel[0].plot(gt_vel_time, gt_v_x, label='Ground truth', color = 'red')
ax_vel[0].plot(est_time, est_v_x, label='Estimated', color = 'blue', linewidth=1)
ax_vel[0].set_ylabel('$x$ vel. ($m/s$)', fontsize = 7)
ax_vel[0].set_xlim([0, 25])
ax_vel[0].tick_params(axis='x', labelsize = 6)
ax_vel[0].tick_params(axis='y', labelsize = 6)
ax_vel[0].grid()
# ax_vel[0].text(20, -2.5, 'RMSE: '+'%.4f'%RMSE_v_x, fontsize=9)

ax_vel[1].plot(gt_vel_time, gt_v_y, label='Ground truth', color = 'red')
ax_vel[1].plot(est_time, est_v_y, label='Estimated', color = 'blue', linewidth=1)
ax_vel[1].set_ylabel('$y$ vel. ($m/s$)', fontsize = 7)
ax_vel[1].set_xlim([0, 25])
ax_vel[1].tick_params(axis='x', labelsize = 6)
ax_vel[1].tick_params(axis='y', labelsize = 6)
ax_vel[1].grid()
# ax_vel[1].text(20, -1.5, 'RMSE: '+'%.4f'%RMSE_v_y, fontsize=9)

ax_vel[2].plot(gt_vel_time, gt_v_z, label='Ground truth', color = 'red')
ax_vel[2].plot(est_time, est_v_z, label='Estimated', color = 'blue', linewidth=1)
ax_vel[2].set_ylabel('$z$ vel. ($m/s$)', fontsize = 7)
ax_vel[2].set_xlim([0, 25])
ax_vel[2].tick_params(axis='x', labelsize = 6)
ax_vel[2].tick_params(axis='y', labelsize = 6)
ax_vel[2].grid()
ax_vel[2].legend(bbox_to_anchor=(-0.1, -0.5), loc = 'lower left', fontsize = 7, ncol=2)
ax_vel[2].set_xlabel('Time (seconds)', fontsize = 7)
# ax_vel[2].text(20, -1.5, 'RMSE: '+'%.4f'%RMSE_v_z, fontsize=9)


# Angular Velocity Plot
fig_omg, ax_omg = plt.subplots(3,1)
fig_omg.set_size_inches(w=5.5, h=4.5) #(w=3.5, h=4)
fig_omg.suptitle('Quadrotor Angular Velocity w.r.t. Body Frame')

ax_omg[0].plot(gt_vel_time, gt_w_x, label='Ground truth', color = 'red')
ax_omg[0].plot(est_time, est_w_x, label='Estimated', color = 'blue', linewidth=1)
ax_omg[0].set_ylabel('${\omega}_x$ ($rad/s$)', fontsize = 7)
ax_omg[0].set_xlim([0, 25])
ax_omg[0].tick_params(axis='x', labelsize = 6)
ax_omg[0].tick_params(axis='y', labelsize = 6)
ax_omg[0].grid()
# ax_omg[0].text(20, -2.5, 'RMSE: '+'%.4f'%RMSE_w_x, fontsize=9)

ax_omg[1].plot(gt_vel_time, gt_w_y, label='Ground truth', color = 'red')
ax_omg[1].plot(est_time, est_w_y, label='Estimated', color = 'blue', linewidth=1)
ax_omg[1].set_ylabel('${\omega}_y$ ($rad/s$)', fontsize = 7)
ax_omg[1].set_xlim([0, 25])
ax_omg[1].tick_params(axis='x', labelsize = 6)
ax_omg[1].tick_params(axis='y', labelsize = 6)
ax_omg[1].grid()
# ax_omg[1].text(20, -1.5, 'RMSE: '+'%.4f'%RMSE_w_y, fontsize=9)

ax_omg[2].plot(gt_vel_time, gt_w_z, label='Ground truth', color = 'red')
ax_omg[2].plot(est_time, est_w_z, label='Estimated', color = 'blue', linewidth=1)
ax_omg[2].set_ylabel('${\omega}_z$ ($rad/s$)', fontsize = 7)
ax_omg[2].set_xlim([0, 25])
ax_omg[2].tick_params(axis='x', labelsize = 6)
ax_omg[2].tick_params(axis='y', labelsize = 6)
ax_omg[2].grid()
ax_omg[2].legend(bbox_to_anchor=(-0.1, -0.5), loc = 'lower left', fontsize = 7, ncol=2)
ax_omg[2].set_xlabel('Time (seconds)', fontsize = 7)
# ax_omg[2].text(20, -0.4, 'RMSE: '+'%.4f'%RMSE_w_z, fontsize=9)

plt.show()
