% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);

% Connect to physical robot
robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
% Note -- this is how long particular commands that you send to the robot "last"
% before the robot goes limp. Here, we ensure they last for 1 second.
robot.setCommandLifetime(5);
% Load saved control gains, and set these on the robot. These can be tuned to
% improve accuracy, but you should be very careful when doing so.
gains = load('jenga_gains.mat');
gains.jenga_gains.positionKp = [1 4 5 2 2];
gains.jenga_gains.positionKd = [0.1 0.1 0.1 .1 .1];
gains.jenga_gains.velocityKp = [4 4 4 4 4];
gains.jenga_gains.torqueKd = [3 3 3 3 3];
gains.jenga_gains.torqueKd = [.01 .01 .01 .01 .01];
robot.set('gains', gains.jenga_gains);

% % %% Connect to gripper, and initialize some settings properly
% % gripper = HebiLookup.newGroupFromNames('16384','gripper');
% % gripper.setCommandLifetime(0);
% % gripper.setFeedbackFrequency(100);
% 
% warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
% disp('');
% input('Once ready, press "enter" to continue...','s');

%% Get initial position
fbk = robot.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot.startLog('file', fullfile(currentDir, 'robot_data'));
%% command frequency, in Hz
frequency = 100;

% t0 = robot.getNextFeedback().time;
% t = 0;
% while t < 10
%     fb = robot.getNextFeedback();
%     %t = fbk.time - t0;
%     t = t + 1;
%     pause(1);
% end

%% Stop logging, and plot results
robot.stopLog();
hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));

% function [position, velocity, torque] = readLog(hebilog)
%     %group = HebiUtils.newGroupFromLog(hebiLogFile);
%     %fbk = group.getNextFeedback();
%     
%     %while isempty(group.getInfo()) && ~isempty(fbk)
%     %    fbk = group.getNextFeedback(fbk);
%     %end
%     position = hebilog.position;
%     velocity = hebilog.velocity;
%     torque = hebilog.torque;
%end
%          theta1   theta2 ...
position = [0.914 0.7607 1.674 0.904 0.1386];
cmd = CommandStruct();
cmd.position = position
robot.set(cmd);
change  = 120;
while true
    caseInput = input("enter case #: ");
    switch caseInput
        case 0
            position(1) = position(1) + pi/change; 
        case 1
            position(1) = position(1) - pi/change;
        case 2
            position(2) = position(2) + pi/change;
        case 3
            position(2) = position(2) - pi/change;
        case 4
            position(3) = position(3) + pi/change;
        case 5
            position(3) = position(3) - pi/change;
        case 6
            position(4) = position(4) + pi/change;
        case 7
            position(4) = position(4) - pi/change;
        case 8
            position(5) = position(5) + pi/change;
        case 9
            position(5) = position(5) - pi/change;
    end
    cmd.position = position
    robot.set(cmd);
end 

