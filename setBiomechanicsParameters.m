% ground
ground.length = 3;
ground.width = 1;
ground.height = 0.1;

% friction
footToGround.friction.muk = 1.0;
footToGround.friction.mus = 1.0;
footToGround.velocity_threshold = 0.01;


% my initial values
% footToGround.stiffness = 1e5; % N/m
% footToGround.damping = 2e3; % N/(m/s)

% SIMBICON values, accounting for the fact that we have two contacts
% footToGround.stiffness = 5e5; % N/m
% footToGround.damping = 3e3; % N/(m/s)

% increased values to avoid breaking through the ground
footToGround.stiffness = 5e5; % N/m
footToGround.damping = 6e3; % N/(m/s)

vertical_force_threshold = 1; % threshold for contact

% define weight factors
% according to R. Dumas , L. Cheze, J.-P. Verriest: "Adjustments to McConville et al. and Young et al. body
% segment inertial parameters", Journal of Biomechanics 40 (2007) 543?553
head_mass_factor      = 0.067;
torso_mass_factor     = 0.333;
arm_mass_factor       = 0.024;
forearm_mass_factor   = 0.017;
hand_mass_factor      = 0.006;
pelvis_mass_factor    = 0.142;
thigh_mass_factor     = 0.123;
shank_mass_factor     = 0.048;
foot_mass_factor      = 0.012;

% whole body
body.mass = 80;
body.height = 1.8;

% geometry
geometry.neck_to_top_x = 0;
geometry.neck_to_top_y = 0;
geometry.neck_to_top_z = body.height * 1/18 * 2.5;
geometry.trunk_height = body.height * 1/18 * 7.5;
geometry.trunk_to_neck_x = 0;
geometry.trunk_to_neck_y = 0;
geometry.trunk_to_neck_z = geometry.trunk_height / 2;
geometry.hip_to_trunk_x = 0.15;
geometry.hip_to_trunk_y = 0;
geometry.hip_to_trunk_z = geometry.trunk_height / 2;
geometry.knee_to_hip_x = 0;
geometry.knee_to_hip_y = 0;
geometry.knee_to_hip_z = body.height * 1/18 * 4;
geometry.ankle_to_knee_x = 0;
geometry.ankle_to_knee_y = 0;
geometry.ankle_to_knee_z = body.height * 1/18 * 4;
geometry.floor_to_ankle_z = body.height * 1/18 * 1;
geometry.ankle_to_foot_x = 0;
geometry.ankle_to_foot_y = 0.05;
geometry.ankle_to_foot_z = geometry.floor_to_ankle_z * 0.5;

% feet
foot.height = geometry.floor_to_ankle_z * 0.5;
foot.width = 0.075;
foot.length = 0.2;
foot.mass = body.mass * foot_mass_factor;
foot.heel_width = 0.05;
foot.heel_offset_ap = 0.04;
foot.ball_width = 0.08;
foot.ball_offset_ap = 0.15;
foot.heel_lat_radius = 0.02;
foot.heel_med_radius = 0.02;
foot.ball_med_radius = 0.02;
foot.ball_lat_radius = 0.02;



% leg joints
leg.hip_radius = 0.05;
leg.knee_radius = 0.04;
leg.ankle_radius = 0.03;

% head
head.mass = body.mass * head_mass_factor;
head.radius = geometry.neck_to_top_z * 0.45;
head.com_to_neck = geometry.neck_to_top_z / 2;

% trunk
trunk.mass = body.mass * (torso_mass_factor + pelvis_mass_factor + 2*arm_mass_factor + 2*forearm_mass_factor + 2*hand_mass_factor);
trunk.width = 0.40;
trunk.depth = 0.25;
trunk.height = geometry.hip_to_trunk_z + geometry.trunk_to_neck_z;

% thighs
thigh.mass = body.mass * thigh_mass_factor;
thigh.width = 0.1;
thigh.depth = 0.1;
thigh.height = geometry.knee_to_hip_z;
thigh_com_from_knee = geometry.knee_to_hip_z / 2;
thigh_com_from_hip = geometry.knee_to_hip_z / 2;

% shanks
shank.mass = body.mass * shank_mass_factor;
shank.width = 0.07;
shank.depth = 0.07;
shank.height = geometry.ankle_to_knee_z;
shank_com_from_ankle = geometry.ankle_to_knee_z / 2;
shank_com_from_knee = geometry.ankle_to_knee_z / 2;


% initial condition (some of these values depend on others)
IC.left_hip_angle = 0.1;
IC.left_knee_angle = -0.2;
IC.left_ankle_angle = 0.1;
IC.right_hip_angle = 0.5;
IC.right_knee_angle = 0;
IC.right_ankle_angle = 0;

IC.trunk_pos_y = 0;
IC.trunk_pos_z = geometry.floor_to_ankle_z + geometry.ankle_to_knee_z + geometry.knee_to_hip_z + geometry.hip_to_trunk_z;
IC.trunk_vel_y = 1;
IC.trunk_vel_z = 0;

IC.neck_pos_y = geometry.ankle_to_knee_y + geometry.knee_to_hip_y + geometry.hip_to_trunk_y + geometry.trunk_to_neck_y;
IC.neck_pos_z = geometry.floor_to_ankle_z + geometry.ankle_to_knee_z + geometry.knee_to_hip_z + geometry.hip_to_trunk_z + geometry.trunk_to_neck_z;
IC.neck_vel_y = IC.trunk_vel_y;
IC.neck_vel_z = 0;

IC.hips_pos_y = geometry.ankle_to_knee_y + geometry.knee_to_hip_y;
IC.hips_pos_z = geometry.floor_to_ankle_z + geometry.ankle_to_knee_z + geometry.knee_to_hip_z;
IC.hips_vel_y = IC.trunk_vel_y;
IC.hips_vel_z = 0;

IC.left_ankle_pos_y = 0;
IC.left_ankle_pos_z = geometry.floor_to_ankle_z;
IC.left_ankle_vel_y = IC.trunk_vel_y;
IC.left_ankle_vel_z = 0;

IC.right_ankle_pos_y = sin(IC.right_hip_angle) * (geometry.ankle_to_knee_z + geometry.knee_to_hip_z);
IC.right_ankle_pos_z = IC.hips_pos_z - cos(IC.right_hip_angle) * (geometry.ankle_to_knee_z + geometry.knee_to_hip_z);
IC.right_ankle_vel_y = IC.trunk_vel_y;
IC.right_ankle_vel_z = 0;



