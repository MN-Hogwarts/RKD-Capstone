function [ torque ] = gravityCap(theta)
% get_grav_comp_torques
%
%   Calculates the joint torques required to cancel out effects due to
%   gravity.

% Get information about the robot:

dh_parameters = [0 pi/2 56.05 0; 330.3 0 103.55 0; 254.1 0 -73.05 0; 0 pi/2 91.05 0; 0 0 213.75 0];
link_masses = zeros(5, 1);
joint_masses = zeros(5, 1);
robot = Robot(dh_parameters, link_masses, joint_masses);
% Extract mass of the links, joint, and end effector [kg]
% Extract length of the links
l1 = 35.56;
l2 = 30.48;
m_link_1 = l1*(0.25/60);
m_link_2 = l2*(0.25/60);
m_joint_1 = 0.347;
m_joint_2 = 0.347;
m_joint_3 = 0.347;
m_joint_4 = 0.347;
m_joint_5 = 0.347;
%m_end_effector = 0.347;

% --------------- BEGIN STUDENT SECTION ----------------------------------
% Use the Jacobian to calculate the joint torques to compensate for the
% weights of the joints, links, and end effector (assuming the acceleration
% due to gravity is given be 'gravity', and it is a 2x1 (column) vector).
Jacobians = jacobians_numerical(robot, theta);
JacobiansCOM = jacobians_com(robot, theta);

J_link1 = JacobiansCOM(:,:,2);
J_link2 = JacobiansCOM(:,:,3);
J_joint1 = Jacobians(:,:,1);
J_joint2 = Jacobians(:,:,2);
J_joint3 = Jacobians(:,:,3);
J_joint4 = Jacobians(:,:,4);
J_joint5 = Jacobians(:,:,5);

transJ_link1 = transpose(J_link1);
transJ_link2 = transpose(J_link2);
transJ_joint1 = transpose(J_joint1);
transJ_joint2 = transpose(J_joint2);
transJ_joint3 = transpose(J_joint3);
transJ_joint4 = transpose(J_joint4);
transJ_joint5 = transpose(J_joint5);

F_link1 = [0; m_link_1*9.8; 0; 0; 0; 0];
F_link2 = [0; m_link_2*9.8; 0; 0; 0; 0];
F_joint1 = [0; m_joint_1*9.8; 0; 0; 0; 0];
F_joint2 = [0; m_joint_2*9.8; 0; 0; 0; 0];
F_joint3 = [0; m_joint_3*9.8; 0; 0; 0; 0];
F_joint4 = [0; m_joint_4*9.8; 0; 0; 0; 0];
F_joint5 = [0; m_joint_5*9.8; 0; 0; 0; 0];

torqueLink1 = (transJ_link1 * F_link1);
torqueLink2 = (transJ_link2 * F_link2);

torqueJoint1 = (transJ_joint1 * F_joint1);
torqueJoint2 = (transJ_joint2 * F_joint2);
torqueJoint3 = (transJ_joint3 * F_joint3);
torqueJoint4 = (transJ_joint4 * F_joint4);
torqueJoint5 = (transJ_joint5 * F_joint5);

allTorques = torqueLink1 + torqueLink2 + torqueJoint1 + torqueJoint2 + torqueJoint3 + torqueJoint4 + torqueJoint5;

torque1 = allTorques(1);
torque2 = allTorques(2);
torque3 = allTorques(3);
torque4 = allTorques(4);
torque5 = allTorques(5);

% --------------- END STUDENT SECTION ------------------------------------
% Pack into a more readable format. DO NOT CHANGE!
torque = cat(1, torque1, torque2, torque3, torque4, torque5);
end
