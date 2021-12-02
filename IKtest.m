%dh_parameters = [0 pi/2 56.05 0; 330.3 0 103.55 0; 254.1 0 -73.05 0; 0 pi/2 91.05 0; 0 0 213.75 0];
dh_parameters = [0 pi/2 56.05 0; 330.3 0 0 0; 254.1 0 0 0; 0 pi/2 -121.05 0; 213.75 0 0 0];
%dh_parameters = [56.05 pi/2 0 0; 0 0 330.3 0; 0 0 254.1 0; 121.5 pi/2 0 0; 213.75 0 0 0];
link_masses = zeros(5,1);
joint_masses = zeros(5,1);
robot = Robot(dh_parameters, link_masses, joint_masses);
thetas = [0.548 0.9162 1.27 1.125 0.1107];
thetas(3) = -thetas(3);
% thetas = [0.548 0.9162 -1.27 1.125 0];
thetas(4) = -pi/2 - thetas(2) - thetas(3);
thetas(5) = 0;
%ee = robot.ee([0.548 0.9162 1.27 1.125 0.1107])
ee = robot.ee(thetas)
robot.inverse_kinematics_analytical(ee)
%robot.inverse_kinematics_graddescent([0.1029 0.8373 1.191 1.256 1.681]',ee)
