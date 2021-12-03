function [] = pick_place_sample()
% pick_place_sample
%
% pick and place example code; picks up an object at position "A" on
% the table and moves it to position "B".

%% First, we define a couple of helper functions.  You can break these out into
%% separate files if you wish.

% Define a simple function to actually send a series of points to the
% robot, where the 'trajectory' is a matrix of columns of joint angle
% commands to be sent to 'robot' at approximately 'frequency'.
% Note this also commands velocities, although you can choose to only command
% positions if desired, or to add torques to help compensate for gravity.
function [] = command_trajectory(robot, trajectory, frequency)
  %% Setup reusable structures to reduce memory use in loop
  cmd = CommandStruct();

  % Compute the velocity numerically
  trajectory_vel = diff(trajectory, 1, 2);

  % Command the trajectory
  for i = 1:(size(trajectory, 2) - 1)
    % Send command to the robot (the transposes on the trajectory
    % points turns column into row vector for commands).
    cmd.position = trajectory(:,i)';
    cmd.velocity = trajectory_vel(:,i)' * frequency;
    %gravTorques = gravityCap(trajectory(:,i));
    %cmd.torque = transpose(gravTorques); 
    robot.set(cmd);

    % Wait a little bit to send at ~100Hz.
    pause(1 / frequency);
  end

  % Send the last point, with a goal of zero velocity.
  cmd.position = trajectory(:,end)';
  cmd.velocity = zeros(1, size(trajectory, 1));
  %gravTorques = gravityCap(trajectory(:,end));
  %cmd.torque = transpose(gravTorques); 
  robot.set(cmd);
end

% Convenience function to use to hide the internal logic of starting the suction
function [] = pick(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 1;
  suction_cup.set(suction_cmd);
end

% Convenience function to use to hide the internal logic of stopping the suction
function [] = place(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 0;
  suction_cup.set(suction_cmd);
end

% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);

% Connect to physical robot
robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
% Note -- this is how long particular commands that you send to the robot "last"
% before the robot goes limp. Here, we ensure they last for 1 second.
robot.setCommandLifetime(1);
% Load saved control gains, and set these on the robot. These can be tuned to
% improve accuracy, but you should be very careful when doing so.
gains = load('jenga_gains.mat');
gains.jenga_gains.positionKp = [1 4 5 2 2];
gains.jenga_gains.positionKi = [0 0 0 0 0];
gains.jenga_gains.positionKd = [0.05 0.01 0.05 0 .01];
gains.jenga_gains.positionFF = [0 .05 .05 0 0];
gains.jenga_gains.velocityKp = [.01 .01 .01 .01 .01];
% gains.jenga_gains.velocityKi = [0 0 0 0 0];
% gains.jenga_gains.velocityKd = [0.01 0.01 0.01 0 .01];
% gains.jenga_gains.velocityFF = [0 .05 .05 0 0];
%gains.jenga_gains.velocityKp = [4 4 4 4 4];
%gains.jenga_gains.torqueKd = [3 3 3 3 3];
%gains.jenga_gains.torqueKd = [.01 .01 .01 .01 .01];
robot.set('gains', gains.jenga_gains);
while(~isequal(robot.get('gains').positionKi, gains.jenga_gains.positionKi)) 
	robot.set('gains', gains.jenga_gains);
	pause(.1);
end
robot.get('gains')

%% Connect to gripper, and initialize some settings properly
gripper = HebiLookup.newGroupFromNames('16384','gripper');
gripper.setCommandLifetime(0);
gripper.setFeedbackFrequency(100);

% We define our "position 1", "position 2", and a "midpoint" waypoints
% here. We found these points by calling robot.getNextFeedback() from the MATLAB
% command line, and tweaking the results as necessary.
% We define our "position 1", "position 2", and a "midpoint" waypoints
% here. We found these points by calling robot.getNextFeedback() from the MATLAB
% command line, and tweaking the results as necessary.
%position_1 = [0.1368 0.7534 1.142 1.286 1.669]';
position_1 = [0.1173 0.7194 1.0171 1.2449 -0.8377]';
%position_2 = [[0.9875 0.7185 1.509 0.8329 0.1196]', [0.951 0.7927 1.623 0.9488 0.1436]',[0.9041 0.7482 1.729 0.9803 1.673]'];
% Layer 1 probably
%position_2 = [[1.003 0.7043 1.552 1.04 0.1418]', [0.9671 0.756 1.658 1.066 0.1382]',[0.9109 0.7847 1.6967 1.0336 0.1386]']; 
%position_2 = [[.999 0.7031 1.549 1.0382 0.1418]', [0.9630 0.7548 1.655 1.0642 0.1382]',[0.9109 0.7847 1.6967 1.0336 0.1386]'];
%position_2 = [[0.998 0.7894 1.6878 0.8023 0.1408]', [0.9384 0.7785 1.6007 0.7661 0.1408]',[0.8795 0.7208 1.5225 0.7804 0.1407]'];
middle2 = [0.9334    0.7747    1.6609    0.9482    0.1205];
middle2(4) = middle2(2) - middle2(3) + 1.9; % Angle pointing down: -1.9ish?
middle2 = offsetWaypoint(middle2, 7, 3);
middle2 = offsetWaypoint(middle2, 5, 2);
%middle2 = offsetWaypoint(middle2, 2, 1);
s1r2 = offsetWaypoint(middle2, -28, 1);
s1r2 = offsetWaypoint(s1r2, -7, 2);
s2r2 = offsetWaypoint(middle2, 28, 1);
s2r2 = offsetWaypoint(s2r2, 7, 2);
middle1 = [0.9671 0.756 1.658 1.066 0.1382];
middle1(5) = middle1(5) - 0.15;
middle1(4) = middle1(2) - middle1(3) + 1.9;
middle1 = offsetWaypoint(middle1, 5, 1);
middle1 = offsetWaypoint(middle1, 2, 2);
middle1 = offsetWaypoint(middle1, 3, 3);
s1r1 = offsetWaypoint(middle1, 19, 2);
s1r1 = offsetWaypoint(s1r1, -1, 1);
s1r1 = offsetWaypoint(s1r1, 2, 3);
s2r1 = offsetWaypoint(middle1, -17, 2);
s2r1 = offsetWaypoint(s2r1, 5, 1);
s2r1 = offsetWaypoint(s2r1, -2, 3);
position_2 = [s1r1', middle1',s2r1', ...
    s1r2', middle2', s2r2'];

% We keep the last joint equal to the first to ensure the block does not rotate
% as we move. Note this joint points in the opposite direction as the base. For
% position 2, we want to rotate 1/4 turn, so we add pi/2.
position_1(5) = position_1(1);
position_2(5,1) = position_2(1,1) + pi/2;
position_2(5,2) = position_2(1,2) + pi/2;
position_2(5,3) = position_2(1,3) + pi/2;
position_2(5,4) = position_2(1,4);
position_2(5,5) = position_2(1,5);
position_2(5,6) = position_2(1,6);

% Add a midpoint for the trajectory, so the robot does not just drag the piece
% across the table.
midpoint = [0.548 0.9162 1.27 1.125 0.1107]';
midpoint(5) = position_1(5)*0.5 + position_2(5,1)*0.5;

% Create a set of "approach" angles that let us have a slow "final approach" to
% the actual pick and place location.  This can increase accuracy and reduce
% issues where straight-line-configuration-space trajectories make the end
% effector hit the table
position_1_approach = [0.1029 0.8627 1.1359 1.1754 1.681]';
position_2_approach = [0.9149 0.8638 1.637 0.9679 0.1107]';

position_1_approach(5) = position_1(5);
position_2_approach(5) = position_2(5,1);

%% Get initial position
fbk = robot.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot.startLog('file', fullfile(currentDir, 'robot_data'));
%% command frequency, in Hz
frequency = 100;

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');

%% Moves the robot from the initial position to the first waypoint over 4
%% seconds.  We break this into 3 seconds to make most of the motion, and 1 for
%% the final approach.
trajectory = trajectory_spline([initial_thetas midpoint position_1_approach], [0, .5, 1], frequency);
command_trajectory(robot, trajectory, frequency);
pick(gripper);
trajectory = trajectory_spline([position_1_approach position_1], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

layerH = 14;
for stack = 1:3
    for ind = 1:6
        block_pos = position_2(:,ind);
        if (stack > 1)
            block_pos = offsetWaypoint(position_2(:,ind), (stack-1)*2 * layerH, 3)';
        end
%         if (stack == 2 && ind <= 3) % row 3
%             block_pos = offsetWaypoint(block_pos, 4, 1)';
%             if (ind == 3) % block 3
%                 block_pos = offsetWaypoint(block_pos, -7, 2)';
%             end
%         end
%         if (stack > 1 && ind == 3) % row 3 and 5, block 3
%             block_pos = offsetWaypoint(block_pos, 2, 1)';
%         end
        if (stack == 1 && ind == 1) % row 1, block 1
            block_pos = offsetWaypoint(block_pos, 4, 1)';
        end
        %approach_pos = offsetWaypoint(block_pos, 40, 3)';
        approach_pos = block_pos; approach_pos(2) = approach_pos(2) + 0.1;
    
        % %% Pick up the object at position 1.  We pause to let the robot stabilize before
        % %% moving. NOTE: If you pause more than the length of the command lifetime you
        % %% set above, then the robot "goes limp" because the previous position and/or
        % %% velocity and torque commands "expire".
%         pick(gripper);
%         pause(0.75);
    
        %% Move to the second waypoint over 4 seconds, with special "retract" and
        %% "approach" motions that are done more slowly.
        trajectory = trajectory_spline([position_1 position_1_approach], [0, 1], frequency);
        command_trajectory(robot, trajectory, frequency);
        trajectory = trajectory_spline([position_1_approach midpoint approach_pos], [0, .5, 1.5], frequency);
        command_trajectory(robot, trajectory, frequency);
        trajectory = trajectory_spline([approach_pos block_pos], [0, 1], frequency);
        command_trajectory(robot, trajectory, frequency);
        
        % %% Place the object
        place(gripper);
        pause(0.75);
        
        trajectory = trajectory_spline([block_pos approach_pos], [0, 1], frequency);
        command_trajectory(robot, trajectory, frequency);
        
        if (stack ~= 3 || ind ~= 6) % Not the last block
            %% Move back to position 1.
            trajectory = trajectory_spline([approach_pos midpoint position_1_approach], [0, .5, 1.5], frequency);
            command_trajectory(robot, trajectory, frequency);
            pick(gripper);
            trajectory = trajectory_spline([position_1_approach position_1], [0, 1], frequency);
            command_trajectory(robot, trajectory, frequency);
        else % After last block
            midpoint(1) = midpoint(1) - .3;
            trajectory = trajectory_spline([approach_pos midpoint], [0, .5], frequency);
            command_trajectory(robot, trajectory, frequency);            
        end
    end
end

%% Stop logging, and plot results
robot.stopLog();

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));

% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');

end
