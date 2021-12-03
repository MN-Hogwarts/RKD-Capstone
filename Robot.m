classdef Robot
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties %(SetAccess = 'immutable')
        dof
        link_masses
        joint_masses
        dh_parameters
    end
    
    methods
        %% Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot(dh_parameters, link_masses, joint_masses)
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(dh_parameters, 2) ~= 4
                error('Invalid dh_parameters: Should be a dof x 4 matrix, is %dx%d.', size(dh_parameters, 1), size(dh_parameters, 2));
            end
            
            if size(link_masses, 2) ~= 1
                error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
            end
            
            if size(joint_masses, 2) ~= 1
                error('Invalid joint_masses: Should be a column vector.');
            end
            
            robot.dof = size(dh_parameters, 1);
            
            if size(joint_masses, 1) ~= robot.dof
                error('Invalid number of joint masses: should match number of degrees of freedom. Did you forget the base joint?');
            end
            
            if size(link_masses, 1) ~= robot.dof
                error('Invalid number of link masses: should match number of degrees of freedom. Did you forget the base joint?');
            end
            
            robot.dh_parameters = dh_parameters;
            robot.link_masses = link_masses;
            robot.joint_masses = joint_masses;
        end
        
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        
        %% Foward Kinematics        
        function frames = forward_kinematics(robot, thetas)
        
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            frames = zeros(4,4,robot.dof);
            n = robot.dof;
            
            %dh_parameters = robot.dh_parameters;
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 3x3 matrix frames(:,:,i).
            
            % The transform from the end effector to the base frame (H^0_i) is
            % given by the 3x3 matrix frames(:,:,end).
                
            a1 = robot.dh_parameters(1, 1);
            alpha1 = robot.dh_parameters(1, 2);
            d1 = robot.dh_parameters(1, 3);
            theta1 = robot.dh_parameters(1, 4) + thetas(1);
            
            frames(:,:,1) = [cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1); 
                sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1); 
                0 sin(alpha1) cos(alpha1) d1;
                0 0 0 1];
            
            for i = 2:n
                a = robot.dh_parameters(i, 1);
                alpha = robot.dh_parameters(i, 2);
                d = robot.dh_parameters(i, 3);
                theta = robot.dh_parameters(i, 4) + thetas(i);
            
                currFrame = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); 
                    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); 
                    0 sin(alpha) cos(alpha) d; 
                    0 0 0 1];
                frames(:,:,i) = frames(:,:,i-1) * currFrame;
            end
        end
        
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
        
        % Returns [x; y; z; psi; theta; phi] for the end effector given a
        % set of joint angles. Remember that psi is the roll, theta is the
        % pitch, and phi is the yaw angle.
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
            
            % Extract the components of the end_effector position and
            % orientation.
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            x = H_0_ee(1,4,end);
            y = H_0_ee(2,4,end);
            z = H_0_ee(3,4,end);
            angles = rotm2eul(H_0_ee(1:3, 1:3)); % download the library later
            % --------------- END STUDENT SECTION ------------------------------------
            % Pack them up nicely.
            ee = [x; y; z; angles(3); angles(2); angles(1)];
        end
        
        % Shorthand for returning the end effector position and orientation.
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        %% Jacobians
        function jacobians = jacobians_com(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof);
            epsilon = 0.001;
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            for joint = 1 : robot.dof
                % TODO perturb the FK by 'epsilon' in this joint, and find the
                % origin of each frame.
                dx = transpose(zeros(1,robot.dof));
                dx(joint) = epsilon;
                thetasPlus = thetas + dx;
                thetasMinus = thetas - dx;
                
                dParam = robot.dh_parameters(joint, 3);
                robot.dh_parameters(joint, 3) = dParam/2;
                
                left = fk(robot, thetasPlus);
                right = fk(robot, thetasMinus);
                leftAngles = rotm2eul(left(1:3, 1:3));
                rightAngles = rotm2eul(right(1:3, 1:3));
                
                robot.dh_parameters(joint, 3) = dParam;
                
                for frame = 1 : (robot.dof)
                    gxLeft = left(1, 4, frame);
                    gxRight = right(1, 4, frame);
                    gyLeft = left(2, 4, frame);
                    gyRight = right(2, 4, frame);
                    gzLeft = left(2, 4, frame);
                    gzRight = right(2, 4, frame);
                    gPhiLeft = leftAngles(1);
                    gPhiRight = rightAngles(1);
                    gThetaLeft = leftAngles(2);
                    gThetaRight = rightAngles(2);
                    gPsiLeft = leftAngles(3);
                    gPsiRight = rightAngles(3);
                
                    jacobians(1, joint, frame) = (gxLeft - gxRight)/(2*epsilon);
                    
                    jacobians(2, joint, frame) = (gyLeft - gyRight)/(2*epsilon);
                    
                    jacobians(3, joint, frame) = (gzLeft - gzRight)/(2*epsilon);
                    
                    jacobians(4, joint, frame) = (gPsiLeft - gPsiRight)/(2*epsilon);
                    
                    jacobians(5, joint, frame) = (gThetaLeft - gThetaRight)/(2*epsilon);
                    
                    jacobians(6, joint, frame) = (gPhiLeft - gPhiRight)/(2*epsilon);
                end
            end
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function jacobians = jacobians_numerical(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof);
            epsilon = 0.001;
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            for joint = 1 : robot.dof
                % TODO perturb the FK by 'epsilon' in this joint, and find the
                % origin of each frame.
                dx = transpose(zeros(1,robot.dof));
                dx(joint) = epsilon;
                thetasPlus = thetas + dx;
                thetasMinus = thetas - dx;
                
                %robot.dh_parameters(joint, 3);
                left = fk(robot, thetasPlus);
                right = fk(robot, thetasMinus);
                leftAngles = rotm2eul(left(1:3, 1:3));
                rightAngles = rotm2eul(right(1:3, 1:3));
                
                %robot.dh_parameters(joint, 3) *=2;
                
                for frame = 1 : (robot.dof)
                    gxLeft = left(1, 4, frame);
                    gxRight = right(1, 4, frame);
                    gyLeft = left(2, 4, frame);
                    gyRight = right(2, 4, frame);
                    gzLeft = left(2, 4, frame);
                    gzRight = right(2, 4, frame);
                    gPhiLeft = leftAngles(1);
                    gPhiRight = rightAngles(1);
                    gThetaLeft = leftAngles(2);
                    gThetaRight = rightAngles(2);
                    gPsiLeft = leftAngles(3);
                    gPsiRight = rightAngles(3);
                
                    jacobians(1, joint, frame) = (gxLeft - gxRight)/(2*epsilon);
                    
                    jacobians(2, joint, frame) = (gyLeft - gyRight)/(2*epsilon);
                    
                    jacobians(3, joint, frame) = (gzLeft - gzRight)/(2*epsilon);
                    
                    jacobians(4, joint, frame) = (gPsiLeft - gPsiRight)/(2*epsilon);
                    
                    jacobians(5, joint, frame) = (gThetaLeft - gThetaRight)/(2*epsilon);
                    
                    jacobians(6, joint, frame) = (gPhiLeft - gPhiRight)/(2*epsilon);
                end
            end
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function jacobians = jacobians_analytical(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % TODO build up the jacobian using the analytical
            % convention from lecture
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        
        %% Inverse Kinematics
        
        function thetas = inverse_kinematics_graddescent(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end
            
            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            thetas = initial_thetas;
            
            % Step size for gradient update
            step_size = 0.0000005;
            
            % Once the norm (magnitude) of the computed gradient is smaller than
            % this value, we stop the optimization
            stopping_condition = 0.00005;
            
            % Also, limit to a maximum number of iterations.
            max_iter = 50000;
            num_iter = 0;
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % Run gradient descent optimization
            while (num_iter < max_iter)
                
                % Compute the gradient for either an [x;y;z] goal or an
                % [x; y; z; psi; theta; phi] goal, using the current value of 'thetas'.
                % TODO fill in the gradient of the squared distance cost function
                % HINT use the answer for theory question 2, the
                % 'robot.end_effector' function, and the 'robot.jacobians'
                % function to help solve this problem
                jacobians = robot.jacobians_numerical(thetas);
                jacobianEnd = jacobians(:,:,robot.dof);
                endEff = robot.end_effector(thetas);
                endEff1 = [0; 0];
                endEff2 = [0; 0];
                ee = robot.end_effector(thetas);
                if (size(goal_position, 1) == 2) % [x;y] goal
                    %cost_gradient = zeros(robot.dof, 1);
                    endEff1(1) = ee(1);
                    endEff1(2) = ee(2);
                    cost_gradient = (transpose(jacobianEnd(1:2,:))*(endEff1 - goal_position));
                else % [x;y;theta] goal
                    %cost_gradient = zeros(robot.dof, 1);
                    endEff2(1) = ee(1);
                    endEff2(2) = ee(2);
                    endEff2(3) = ee(3);
                    cost_gradient = (transpose(jacobianEnd)*(endEff2 - goal_position));
                end
                % Update 'thetas'
                % TODO
                thetas = thetas - step_size*cost_gradient;
                % Check stopping condition, and return if it is met.
                % TODO
                if (norm(cost_gradient) < stopping_condition)
                    return
                end
                num_iter = num_iter + 1;
            end
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function cost = cost_function(robot, thetas, goal_position)
            % Cost function for fmincon
            current_pose = robot.ee(thetas);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            % --------------- END STUDENT SECTION ------------------------------------
            
        end
        
        function thetas = inverse_kinematics_numopt(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function. Using built in optimization (fmincon)
            
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end
            
            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            
            fun = @(thetas)robot.cost_function(thetas, goal_position);
            
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function thetas = inverse_kinematics_analytical(robot, goal_position)
            % Returns the joint angles using an analytical approach to
            % inverse kinematics
            % Note: Kinematics Decoupling might be very useful for this
            % question
            % goal_position: [x; y; z; psi; theta; phi]
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            %if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
            %    error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            %end
            
            v1 = robot.dh_parameters(2,1); %330.3
            v2 = robot.dh_parameters(3,1); %254.1
            d = 130;
            thetas = zeros(robot.dof, 1);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            thetas(1) = atan2(goal_position(2),goal_position(1)) - ...
                       asin(d/sqrt(goal_position(1)^2 + goal_position(2)^2));
            
            xrr = sqrt((goal_position(2) - d * cos(thetas(1)))^2 + ...
                    ((goal_position(1)) + d * sin(thetas(1)))^2);
            zrr = goal_position(3) - robot.dh_parameters(1,3) + robot.dh_parameters(5,1);

            num3 = (xrr^2 + zrr^2 - v1^2 - v2^2)/(2*v1*v2);
%             num1 = ((goal_position(3) + 157.7)^2 + goal_position(1)^2 - v1^2 - v2^2)/(2*v1*v2)
            thetas(3) = -acos(num3); % negative because elbow always down
            thetas(2) = atan2(zrr,xrr) - atan2(v2*sin(thetas(3)),(v1+v2*cos(thetas(3))));
%             thetas(1) = atan2(goal_position(2),goal_position(1)) - ...
%                         atan2(121.05, v1*thetas(2)+v2*(thetas(2) + thetas(3)));
            thetas(4) = -pi/2 - thetas(2) - thetas(3);
            thetas(5) = 0;

            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function thetas = trajectory_to_thetas(robot, trajectory)
            thetas(:, 1) = robot.inverse_kinematics_analytical(trajectory(:, 1));
            for theta_col=2:size(trajectory, 2)
                thetas(:, theta_col) = robot.inverse_kinematics_analytical(trajectory(:, theta_col));
            end
        end
        
    end
end