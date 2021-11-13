% set biomechanics parameters
setBiomechanicsParameters

% set initial condition -- this represents a simple inverse kinematics solution done by hand to get the body upright 
% with the foot touching the ground. Trunk is moving forward at 1 m/s. This assumes that the trunk segment is vertical
% and the left foot segment is horizontal.
IC.left_hip_angle = 0.1;
IC.left_knee_angle = -0.2;
IC.left_ankle_angle = 0.1;
IC.right_hip_angle = 0.3;
IC.right_knee_angle = -0.9;
IC.right_ankle_angle = 0.6;

IC.trunk_pos_y = 0;
IC.trunk_pos_z = geometry.floor_to_ankle_z ...
    + cos(IC.left_ankle_angle) * geometry.ankle_to_knee_z ...
    + cos(IC.left_ankle_angle + IC.left_knee_angle) * geometry.knee_to_hip_z ...
    + cos(IC.left_ankle_angle + IC.left_knee_angle + IC.left_hip_angle) * geometry.hip_to_trunk_z;
IC.trunk_vel_y = 1;
IC.trunk_vel_z = 0;

% create rigid body tree model
createWalkerWithFixedStanceFootModel;

% use rigid body tree model to get initial torques that cancel out gravitation
theta_init = [IC.left_ankle_angle; IC.left_knee_angle; IC.left_hip_angle; IC.right_hip_angle; IC.right_knee_angle; IC.right_ankle_angle];
theta_dot_init = [0; 0; 0; 0; 0; 0];
tau = inverseDynamics(model, theta_init, theta_dot_init); 

% define simulation time
total_time = 0.5;

% define simulation input
dt = 0.001; % sampling rate for input
time = dt : dt : total_time;
omega = 1;
shift = sin(2 * pi * time * omega) * 0.1;
shift_vel = 2 * pi * omega * cos(2 * pi * time * omega) * 0.1;
shift_acc = (2 * pi * omega)^2 * -sin(2 * pi * time * omega) * 0.1;
simin.ground_pos = [time; shift]';
simin.ground_vel = [time; shift_vel]';
simin.ground_acc = [time; shift_acc]';
