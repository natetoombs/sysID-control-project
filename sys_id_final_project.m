%% ENAE788M - Assignment 3 - Sydrak Abdi
clear; close all; clc;

% Collecting Data
rb_orient_test = rosbag('20211012_orientationctrl_test.bag');
rb_sys_id_long = rosbag('20211028_sysid_longitudinal.bag');
rb_sys_id_yaw  = rosbag('20211028_sysid_yaw_heave.bag');

%  Reading relevant messages
read_bag = rb_orient_test;

mess_data_imu       = select(read_bag,'Topic','/mavros/imu/data');
mess_data_onbrd_vel = select(read_bag,'Topic','/mavros/local_position/velocity_body');
mess_data_inputs    = select(read_bag,'Topic','/mavros/rc/in');
mess_gt             = select(read_bag,'Topic','/vicon/m500_joec/m500_joec');

% Converting messages into structs
struct_imu       = readMessages(mess_data_imu,'DataFormat','struct');
struct_onbrd_vel = readMessages(mess_data_onbrd_vel,'DataFormat','struct');
struct_inputs    = readMessages(mess_data_inputs,'DataFormat','struct');
struct_gt        = readMessages(mess_gt,'DataFormat','struct');

% Populating Vectors with struct data

% IMU data
comp_imu_orientation_X  = cellfun(@(m) double(m.Orientation.X),struct_imu);
comp_imu_orientation_Y  = cellfun(@(m) double(m.Orientation.Y),struct_imu);
comp_imu_orientation_Z  = cellfun(@(m) double(m.Orientation.Z),struct_imu);
comp_imu_orientation_W  = cellfun(@(m) double(m.Orientation.W),struct_imu);
data_imu_orientation = [comp_imu_orientation_W,comp_imu_orientation_X,comp_imu_orientation_Y,comp_imu_orientation_Z];

comp_imu_ang_vel_X  = cellfun(@(m) double(m.AngularVelocity.X),struct_imu);
comp_imu_ang_vel_Y  = cellfun(@(m) double(m.AngularVelocity.Y),struct_imu);
comp_imu_ang_vel_Z  = cellfun(@(m) double(m.AngularVelocity.Z),struct_imu);
data_imu_ang_vel = [comp_imu_ang_vel_X,comp_imu_ang_vel_Y,comp_imu_ang_vel_Z];

comp_imu_lin_vel_X  = cellfun(@(m) double(m.LinearAcceleration.X),struct_imu);
comp_imu_lin_vel_Y  = cellfun(@(m) double(m.LinearAcceleration.Y),struct_imu);
comp_imu_lin_vel_Z  = cellfun(@(m) double(m.LinearAcceleration.Z),struct_imu);
data_imu_lin_vel = [comp_imu_lin_vel_X,comp_imu_lin_vel_Y,comp_imu_lin_vel_Z];

data_imu_time = mess_data_imu.MessageList.Time;
data_imu_time = data_imu_time - data_imu_time(1);

% Onboard Velocity Data
comp_onbrd_lin_vel_X  = cellfun(@(m) double(m.Twist.Linear.X),struct_onbrd_vel);
comp_onbrd_lin_vel_Y  = cellfun(@(m) double(m.Twist.Linear.Y),struct_onbrd_vel);
comp_onbrd_lin_vel_Z  = cellfun(@(m) double(m.Twist.Linear.Z),struct_onbrd_vel);
data_onbrd_lin_vel = [comp_onbrd_lin_vel_X,comp_onbrd_lin_vel_Y,comp_onbrd_lin_vel_Z];

comp_onbrd_ang_vel_X  = cellfun(@(m) double(m.Twist.Angular.X),struct_onbrd_vel);
comp_onbrd_ang_vel_Y  = cellfun(@(m) double(m.Twist.Angular.Y),struct_onbrd_vel);
comp_onbrd_ang_vel_Z  = cellfun(@(m) double(m.Twist.Angular.Z),struct_onbrd_vel);
data_onbrd_ang_vel = [comp_onbrd_ang_vel_X,comp_onbrd_ang_vel_Y,comp_onbrd_ang_vel_Z];

comp_onbrd_vel_time_sec = cellfun(@(m) double(m.Header.Stamp.Sec),struct_onbrd_vel);
comp_nbrd_vel_time_nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),struct_onbrd_vel)*1e-9;
data_onbrd_time = comp_onbrd_vel_time_sec + comp_nbrd_vel_time_nsec;
data_onbrd_time = data_onbrd_time - data_onbrd_time(1);

% Controller Inputs
data_input_thrt  = cellfun(@(m) double(m.Channels(1)),struct_inputs);
data_input_roll  = cellfun(@(m) double(m.Channels(2)),struct_inputs);
data_input_pitch = cellfun(@(m) double(m.Channels(3)),struct_inputs);
data_input_yaw   = cellfun(@(m) double(m.Channels(4)),struct_inputs);

comp_input_time_sec = cellfun(@(m) double(m.Header.Stamp.Sec),struct_inputs);
comp_input_time_nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),struct_inputs)*1e-9;
data_input_time = comp_input_time_sec + comp_input_time_nsec;
data_input_time = data_input_time - data_input_time(1);

% Ground Truth Positions and Orientation
comp_gt_translation_X  = cellfun(@(m) double(m.Transform.Translation.X),struct_gt);
comp_gt_translation_Y  = cellfun(@(m) double(m.Transform.Translation.Y),struct_gt);
comp_gt_translation_Z  = cellfun(@(m) double(m.Transform.Translation.Z),struct_gt);
data_gt_position = [comp_gt_translation_X,comp_gt_translation_Y,comp_gt_translation_Z];

comp_gt_rotation_X  = cellfun(@(m) double(m.Transform.Rotation.X),struct_gt);
comp_gt_rotation_Y  = cellfun(@(m) double(m.Transform.Rotation.Y),struct_gt);
comp_gt_rotation_Z  = cellfun(@(m) double(m.Transform.Rotation.Z),struct_gt);
comp_gt_rotation_W  = cellfun(@(m) double(m.Transform.Rotation.W),struct_gt);
data_gt_orientation = [comp_gt_rotation_W,comp_gt_rotation_X,comp_gt_rotation_Y,comp_gt_rotation_Z];

comp_gt_time_sec = cellfun(@(m) double(m.Header.Stamp.Sec),struct_gt);
comp_gt_time_nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),struct_gt)*1e-9;
data_gt_time = comp_gt_time_sec + comp_gt_time_nsec;
data_gt_time = data_gt_time - data_gt_time(1);

% Resampling data to match IMU rate

% Inputs:

time_interp = linspace(0,data_imu_time(end),8000);
dt = time_interp(2) - time_interp(1);

data_input_thrt    = interp1(data_input_time, data_input_thrt, time_interp, 'spline').';
data_input_roll    = interp1(data_input_time, data_input_roll, time_interp, 'spline').';
data_input_pitch   = interp1(data_input_time, data_input_pitch, time_interp, 'spline').';
data_input_yaw     = interp1(data_input_time, data_input_yaw, time_interp, 'spline').';
data_input_heave   = data_input_yaw(0.5*end:end);
data_input_yaw     = data_input_yaw(1:0.5*end);

% Outputs:
data_onbrd_ang_vel = interp1(data_onbrd_time, data_onbrd_ang_vel, time_interp, 'spline');
data_onbrd_lin_vel = interp1(data_onbrd_time, data_onbrd_lin_vel, time_interp, 'spline');

data_imu_orientation = interp1(data_imu_time, data_imu_orientation, time_interp, 'spline');
data_imu_ang_vel = interp1(data_imu_time, data_imu_ang_vel, time_interp, 'spline');
data_imu_lin_vel = interp1(data_imu_time, data_imu_lin_vel, time_interp, 'spline');

EUL = quat2eul(data_imu_orientation);

% ROLL
data_input_roll;
data_output_roll_lat_vel = -data_onbrd_lin_vel(:,2);
data_output_roll_angle = EUL(:,3);

% PITCH
data_input_pitch;
data_output_pitch_lon_vel = data_onbrd_lin_vel(:,1);
data_output_pitch_angle = EUL(:,2);

t_start = 1000;
t_stop = 4500;
chop = 12;

data_input_roll_trim = data_input_roll((t_start + chop):(t_stop + chop));
% data_input_roll_trim(1:12) = []
%
figure
hold on
Theta_d = (data_input_roll_trim - 1489)*pi/(17*180);

plot(data_output_roll_angle(t_start:t_stop),'b')
plot(Theta_d,'r')


input_roll_chop = data_input_roll_trim;
output_roll_chop = data_output_roll_angle(t_start:t_stop);

% Exporting PX4 Controller Vairables
target_act = select(read_bag,'Topic','/mavros/target_actuator_control');
target_act_struct = readMessages(target_act,'DataFormat','struct');

target_act_data = [];
for ii = 1:length(target_act_struct)
    target_act_data(ii,:) = target_act_struct{ii}.Controls(1:4).';    
end

target_act_time_sec = cellfun(@(m) double(m.Header.Stamp.Sec),target_act_struct);
target_act_time_nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),target_act_struct)*1e-9;
target_act_time = target_act_time_sec + target_act_time_nsec;
target_act_time = target_act_time - target_act_time(1);

data_input_target_act_roll  = interp1(target_act_time, target_act_data(:,1), time_interp, 'spline').';
data_input_target_act_pitch = interp1(target_act_time, target_act_data(:,2), time_interp, 'spline').';
data_input_target_act_yaw   = interp1(target_act_time, target_act_data(:,3), time_interp, 'spline').';
data_input_target_act_heave = interp1(target_act_time, target_act_data(:,4), time_interp, 'spline').';



figure
plot(data_input_target_act_roll)


% Starting System ID Assignment
systemIdentification