function [] = gravity_compensation()
% gravity_compensation
%
%   This function makes the robot "weightless", extering joint torques that
%   counteract the effect of gravity on the system.
%
%   This function connects to a robot and records the feedback data while
%   executing setting appropriate joint torques.
%
%   Note that this should reuse function(s) written in ex_02.
%
%   Make sure that the corresponding robot is plugged in.
%
%   Example:
%      gravity_compensation();
%  

%% Robot Connection
robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
% Set up handler to be called on exception or ctrl-C.
onCleanup(@() cleanup(robot));

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
input('Once ready, press "enter" to continue; press "ctrl-c" when finished...','s');

%% Define variables for gravity:
gravity = [0; -9.8]; % [m/s^2]

%% Setup reusable structures to reduce memory use in loop
cmd = CommandStruct();
tmpFbk = robot.getNextFeedback();

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot.startLog('file', fullfile(currentDir, 'grav_comp'));

%% Loop, sending commands to the robot, until 'ctrl-c' is pressed.
while true
    
    % Read Robot feedback
    fbk = robot.getNextFeedback(tmpFbk);
    theta = fbk.position;

    % --------------- BEGIN STUDENT SECTION ----------------------------------
    % Set the torques that the robot should command (should move the end
    % effector towards 'center_pt_spring').

    cmd.torque = (gravityCap(theta.')).'; % Note: should be a 1x2 (row) vector.

    % --------------- END STUDENT SECTION ------------------------------------

    % Send command to robot; limit velocity to damp out fast motions.
    cmd.velocity = [0,0,0,0,0];
    robot.set(cmd);

    % Wait a little bit; here we'll cap command rates to 100Hz.
    pause(0.1);
end

function cleanup(robot)

robot.stopLog();

hebilog = HebiUtils.convertGroupLog('grav_comp.hebilog');

% Check that the three start points were achieved by calculating minimum
% distance from known good angles.
thresh = 0.1; % [rad]
test_pts = [0 0; ...
            pi/2 0; ...
            pi/4 pi/2; ...
            3*pi/4 -pi/2; ...
            pi 0];
missed_point = false;
for i = 1:size(test_pts,1)
  dist = sqrt(min(sum((hebilog.position - repmat(test_pts(i,:),size(hebilog.position,1),1)).^2,2)));
  if (dist > thresh)
    warning(['Did not get close enough to start point ' num2str(i) '! Please re-run script before submitting!'])
    missed_point = true;
  end
end

if (~missed_point)
    disp('Successfully reached all start points, and created log for submission');
end

end

end
