% %dh_parameters = [0 pi/2 56.05 0; 330.3 0 103.55 0; 254.1 0 -73.05 0; 0 pi/2 91.05 0; 0 0 213.75 0];
% dh_parameters = [0 pi/2 56.05 0; 330.3 0 0 0; 254.1 0 0 0; 0 pi/2 -121.05 0; 213.75 0 0 0];
% %dh_parameters = [56.05 pi/2 0 0; 0 0 330.3 0; 0 0 254.1 0; 121.5 pi/2 0 0; 213.75 0 0 0];
% link_masses = zeros(5,1);
% joint_masses = zeros(5,1);
% robot = Robot(dh_parameters, link_masses, joint_masses);
% thetas = [0.548 0.9162 1.27 1.125 0.1107];
% thetas(3) = -thetas(3);
% % thetas = [0.548 0.9162 -1.27 1.125 0];
% thetas(4) = -pi/2 - thetas(2) - thetas(3);
% thetas(5) = 0;
% %ee = robot.ee([0.548 0.9162 1.27 1.125 0.1107])
% ee = robot.ee(thetas)
% robot.inverse_kinematics_analytical(ee)
% %robot.inverse_kinematics_graddescent([0.1029 0.8373 1.191 1.256 1.681]',ee)
offsetWaypoint([1.0016 0.773 1.6137 0.8377 0.1204], -2, 2)
offsetWaypoint([0.907 0.7392 1.5332 0.791 0.1204], 3, 1)

% [0.9895 0.7205 1.578 0.8976 0.1408]
% [0.9434 0.7374 1.633 0.8993 0.1408]
% [0.904 0.761 1.684 0.8994 0.1408]

% [0.9873 0.7867 1.681 0.7983 0.1408]
% [0.9483 0.7595 1.614 0.7984 0.1408]
% [0.9072 0.7334 1.553 0.7982 0.1407]


