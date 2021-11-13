%  create a rigidBodyTree robot model
model = rigidBodyTree('DataFormat', 'column');
model.Gravity = [0; 0; -9.8067];
base = model.Base;

% create a series of linkages as rigidBody objects
model_left_shank = rigidBody('left shank');
model_left_thigh = rigidBody('left thigh');
model_trunk = rigidBody('trunk');
model_right_thigh = rigidBody('right thigh');
model_right_shank = rigidBody('right shank');
model_right_foot = rigidBody('right foot');
% model_head = rigidBody('head');

% Each rigid body is attached using a revolute joint. Create the rigidBodyJoint objects for each body. 
left_ankle_joint = rigidBodyJoint('left_ankle', 'revolute');
left_knee_joint = rigidBodyJoint('left_knee', 'revolute');
left_hip_joint = rigidBodyJoint('left_hip', 'revolute');
right_hip_joint = rigidBodyJoint('right_hip', 'revolute');
right_knee_joint = rigidBodyJoint('right_knee', 'revolute');
right_ankle_joint = rigidBodyJoint('right_ankle', 'revolute');
% neck_joint = rigidBodyJoint('neck', 'fixed');

% Specify the z-axis as the axis of rotation for all joints
left_ankle_joint.JointAxis = [-1 0 0];
left_knee_joint.JointAxis = [-1 0 0];
left_hip_joint.JointAxis = [-1 0 0];
right_hip_joint.JointAxis = [1 0 0];
right_knee_joint.JointAxis = [1 0 0];
right_ankle_joint.JointAxis = [1 0 0];

% calculate inertia
I_foot_xx = foot.mass/20 * (foot.length^2 + foot.height^2);
I_foot_yy = foot.mass/20 * (foot.width^2 + foot.height^2);
I_foot_zz = foot.mass/20 * (foot.length^2 + foot.width^2);
I_shank_xx = shank.mass/20 * (shank.depth^2 + shank.height^2);
I_shank_yy = shank.mass/20 * (shank.width^2 + shank.height^2);
I_shank_zz = shank.mass/20 * (shank.depth^2 + shank.width^2);
I_thigh_xx = thigh.mass/20 * (thigh.depth^2 + thigh.height^2);
I_thigh_yy = thigh.mass/20 * (thigh.width^2 + thigh.height^2);
I_thigh_zz = thigh.mass/20 * (thigh.depth^2 + thigh.width^2);
I_trunk_xx = trunk.mass/20 * (trunk.depth^2 + trunk.height^2);
I_trunk_yy = trunk.mass/20 * (trunk.width^2 + trunk.height^2);
I_trunk_zz = trunk.mass/20 * (trunk.depth^2 + trunk.width^2);
I_head_xx = 2/5 * head.mass * head.radius^2;
I_head_yy = 2/5 * head.mass * head.radius^2;
I_head_zz = 2/5 * head.mass * head.radius^2;

% calculate inertia around joint using parallel axis theorem (assuming no rotation between coordinate frames)
I_left_shank_xx = I_shank_xx + shank.mass * shank_com_from_ankle^2;
I_left_shank_yy = I_shank_yy + shank.mass * shank_com_from_ankle^2;
I_left_shank_zz = I_shank_zz;
I_left_thigh_xx = I_thigh_xx + thigh.mass * thigh_com_from_knee^2;
I_left_thigh_yy = I_thigh_yy + thigh.mass * thigh_com_from_knee^2;
I_left_thigh_zz = I_thigh_zz;

t_hip_to_trunk = [geometry.hip_to_trunk_x; geometry.hip_to_trunk_y; geometry.hip_to_trunk_z;];
I_trunk_joint = diag([I_trunk_xx, I_trunk_yy, I_trunk_zz]) + trunk.mass * (eye(3)*(t_hip_to_trunk'*t_hip_to_trunk) - t_hip_to_trunk*t_hip_to_trunk');
I_trunk_joint_xx = I_trunk_joint(1, 1);
I_trunk_joint_yy = I_trunk_joint(2, 2);
I_trunk_joint_zz = I_trunk_joint(3, 3);
I_trunk_joint_yz = -I_trunk_joint(2, 3);
I_trunk_joint_xz = -I_trunk_joint(1, 3);
I_trunk_joint_xy = -I_trunk_joint(1, 2);

t_hip_to_head = [geometry.hip_to_trunk_x + geometry.trunk_to_neck_x; geometry.hip_to_trunk_y + geometry.trunk_to_neck_y; geometry.hip_to_trunk_z + geometry.trunk_to_neck_z + head.com_to_neck;];
I_head_joint = diag([I_head_xx, I_head_yy, I_head_zz]) + head.mass * (eye(3)*(t_hip_to_head'*t_hip_to_head) - t_hip_to_head*t_hip_to_head');
I_head_joint_xx = I_head_joint(1, 1);
I_head_joint_yy = I_head_joint(2, 2);
I_head_joint_zz = I_head_joint(3, 3);
I_head_joint_yz = -I_head_joint(2, 3);
I_head_joint_xz = -I_head_joint(1, 3);
I_head_joint_xy = -I_head_joint(1, 2);

I_right_thigh_xx = I_thigh_xx + thigh.mass * thigh_com_from_hip^2;
I_right_thigh_yy = I_thigh_yy + thigh.mass * thigh_com_from_hip^2;
I_right_thigh_zz = I_thigh_zz;
I_right_shank_xx = I_shank_xx + shank.mass * shank_com_from_ankle^2;
I_right_shank_yy = I_shank_yy + shank.mass * shank_com_from_ankle^2;
I_right_shank_zz = I_shank_zz;

t_ankle_to_foot = [geometry.ankle_to_foot_x; geometry.ankle_to_foot_y; geometry.ankle_to_foot_z;];
I_foot_joint = diag([I_foot_xx, I_foot_yy, I_foot_zz]) + foot.mass * (eye(3)*(t_ankle_to_foot'*t_ankle_to_foot) - t_ankle_to_foot*t_ankle_to_foot');
I_foot_joint_xx = I_foot_joint(1, 1);
I_foot_joint_yy = I_foot_joint(2, 2);
I_foot_joint_zz = I_foot_joint(3, 3);
I_foot_joint_yz = -I_foot_joint(2, 3);
I_foot_joint_xz = -I_foot_joint(1, 3);
I_foot_joint_xy = -I_foot_joint(1, 2);


% set mass and inertia
model_left_shank.Mass = shank.mass;
model_left_shank.CenterOfMass = [0 0 shank_com_from_ankle];
model_left_shank.Inertia = [I_left_shank_xx I_left_shank_yy I_left_shank_zz 0 0 0];
model_left_thigh.Mass = thigh.mass;
model_left_thigh.CenterOfMass = [0 0 thigh_com_from_knee];
model_left_thigh.Inertia = [I_left_thigh_xx I_left_thigh_yy I_left_thigh_zz 0 0 0];

% trunk segment alone
% model_trunk.Mass = trunk.mass;
% model_trunk.CenterOfMass = [geometry.hip_to_trunk_x geometry.hip_to_trunk_y geometry.hip_to_trunk_z];
% model_trunk.Inertia = [I_trunk_joint_xx I_trunk_joint_yy I_trunk_joint_zz I_trunk_joint_yz I_trunk_joint_xz I_trunk_joint_xy];

% trunk and head segments combined
model_trunk.Mass = trunk.mass + head.mass;
trunk_com = [geometry.hip_to_trunk_x geometry.hip_to_trunk_y geometry.hip_to_trunk_z];
model_trunk.Inertia = [I_trunk_joint_xx I_trunk_joint_yy I_trunk_joint_zz I_trunk_joint_yz I_trunk_joint_xz I_trunk_joint_xy] + [I_head_joint_xx I_head_joint_yy I_head_joint_zz I_head_joint_yz I_head_joint_xz I_head_joint_xy];
head_com = [geometry.hip_to_trunk_x geometry.hip_to_trunk_y geometry.hip_to_trunk_z + geometry.trunk_to_neck_z + head.com_to_neck];
model_trunk.CenterOfMass = trunk.mass / model_trunk.Mass * trunk_com + head.mass / model_trunk.Mass * head_com;


model_right_thigh.Mass = thigh.mass;
model_right_thigh.CenterOfMass = [geometry.hip_to_trunk_x * 2 0 -thigh_com_from_hip];
model_right_thigh.Inertia = [I_right_thigh_xx I_right_thigh_yy I_right_thigh_zz 0 0 0];
model_right_shank.Mass = shank.mass;
model_right_shank.CenterOfMass = [0 0 -shank_com_from_knee];
model_right_shank.Inertia = [I_right_shank_xx I_right_shank_yy I_right_shank_zz 0 0 0];

model_right_foot.Mass = foot.mass;
model_right_foot.CenterOfMass = [geometry.ankle_to_foot_x geometry.ankle_to_foot_y -geometry.ankle_to_foot_z];
model_right_foot.Inertia = [I_foot_joint_xx I_foot_joint_yy I_foot_joint_zz I_foot_joint_yz I_foot_joint_xz I_foot_joint_xy];

% Set transformations of the joint attachment between bodies.
% Each transformation is based on the dimensions of the previous rigid body length (z-axis). 
setFixedTransform(left_ankle_joint, trvec2tform([0 0 0]))
setFixedTransform(left_knee_joint, trvec2tform([0 0 geometry.ankle_to_knee_z]))
setFixedTransform(left_hip_joint, trvec2tform([0 0 geometry.knee_to_hip_z]))
setFixedTransform(right_hip_joint, trvec2tform([geometry.hip_to_trunk_x * 2 0 0]))
setFixedTransform(right_knee_joint, trvec2tform([0 0 -geometry.knee_to_hip_z]))
setFixedTransform(right_ankle_joint, trvec2tform([0 0 -geometry.ankle_to_knee_z]))

% assemble
model_left_shank.Joint = left_ankle_joint;
addBody(model, model_left_shank, 'base')
model_left_thigh.Joint = left_knee_joint;
addBody(model, model_left_thigh, 'left shank')
model_trunk.Joint = left_hip_joint;
addBody(model, model_trunk, 'left thigh')
model_right_thigh.Joint = right_hip_joint;
addBody(model, model_right_thigh, 'trunk')
model_right_shank.Joint = right_knee_joint;
addBody(model, model_right_shank, 'right thigh')
model_right_foot.Joint = right_ankle_joint;
addBody(model, model_right_foot, 'right shank')

% report and visualize
% showdetails(model)
% interactiveRigidBodyTree(model, "MarkerScaleFactor", 0.25);

