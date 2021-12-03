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
gains.jenga_gains.positionKi = [0 0 0 0 0];
gains.jenga_gains.positionKd = [0.01 0.01 0.01 0 .01];
gains.jenga_gains.positionFF = [0 .05 .05 0 0];
%gains.jenga_gains.velocityKp = [4 4 4 4 4];
%gains.jenga_gains.torqueKd = [3 3 3 3 3];
%gains.jenga_gains.torqueKd = [.01 .01 .01 .01 .01];
robot.set('gains', gains.jenga_gains);
robot.get('gains')

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');

%% Get initial position
fbk = robot.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
%% command frequency, in Hz
frequency = 100;

%          theta1   theta2 ...
position = [0.2003 0.9518 2.319 0.6173 1.203];
cmd = CommandStruct();
cmd.position = position;
robot.set(cmd);
pause(2);
cmd.position = offsetWaypoint(position, 30);
robot.set(cmd);
pause(2);
