function outpoints = offsetWaypoint(waypoint, offset, dim)
    dh_parameters = [0 pi/2 56.05 0; 317.5 0 0 0; 298.45 0 0 0; 0 pi/2 -130 0; 118 0 0 0];
    link_masses = zeros(5,1);
    joint_masses = zeros(5,1);
    robot = Robot(dh_parameters, link_masses, joint_masses);
    thetas = waypoint;
    % Manipulate robot angles to work with IK
    thetas(3) = -thetas(3);
    thetas(4) = -pi/2 - thetas(2) - thetas(3);
    thetas(5) = 0;
    ee = robot.ee(thetas);
    ee(dim) = ee(dim) + offset;
    outpoints = robot.inverse_kinematics_analytical(ee);

    % Undo manipulations done earlier
    outpoints(3) = -outpoints(3);
    outpoints(4) = waypoint(4) + (outpoints(4) - thetas(4));
    outpoints(5) = waypoint(5);
    outpoints = outpoints';
end
