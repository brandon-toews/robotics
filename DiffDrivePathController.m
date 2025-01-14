% drive cript for sending commands to controller node in Isaac sim through
% ros2 bridge
classdef DiffDrivePathController < handle
    properties
        maxLinearVelocity = 0.5    % m/s
        maxAngularVelocity = 1.0   % rad/s
        lookAheadDistance = 0.5    % meters
        goalThreshold = 0.2        % meters
        velPub                     % ROS2 publisher
        currentWaypointIndex = 1   % Current target waypoint in the path
    end
    
    methods
        function obj = DiffDrivePathController(node)
            % Create ROS2 publisher
            obj.velPub = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
        end
        
        function [v, omega] = computeVelocityCommands(obj, robotState, path)
            % If path is empty, stop
            if isempty(path)
                v = 0;
                omega = 0;
                return;
            end
            
            % Extract robot pose
            x = robotState(1);
            y = robotState(2);
            theta = robotState(3);
            
            % Get current target waypoint
            targetPoint = path(obj.currentWaypointIndex, :);
            
            % Compute distance to target
            distance = norm([x, y] - targetPoint);

            endIndex = size(path, 1);

            finishPoint = path(endIndex, :);
            distanceToFinish = norm([x, y] - finishPoint);
            
            % If we've reached the current waypoint, move to next one
            if distance < obj.goalThreshold
                if obj.currentWaypointIndex < endIndex
                    % Calculate next index with increment of 10
                    nextIndex = obj.currentWaypointIndex + 10;
                    
                    % If next index would exceed path length, go to last point
                    if nextIndex > endIndex
                        nextIndex = endIndex;
                    end
                    
                    obj.currentWaypointIndex = nextIndex;
                    disp(['Moving to waypoint ', num2str(obj.currentWaypointIndex)]);
                    targetPoint = path(obj.currentWaypointIndex, :);
                    distance = norm([x, y] - targetPoint);
                end
            end
            
            % Compute desired heading to target
            targetHeading = atan2(targetPoint(2)-y, targetPoint(1)-x);
            headingError = wrapToPi(targetHeading - theta);
            
            % Add dead zone for small heading errors (0.1 radians ≈ 5.7 degrees)
            if abs(headingError) < 0.1
                headingError = 0;
            end
            
            % Make angular velocity proportional to heading error
            K_angular = 0.5; % Proportional gain for angular velocity
            
            % Compute velocities with improved logic
            if abs(headingError) > pi/4  % If heading error is large (>45 degrees)
                % Turn in place
                v = 0;
                omega = K_angular * headingError;
            else
                % Move forward while turning
                v = obj.maxLinearVelocity * cos(headingError);
                omega = K_angular * headingError;
            end
            
            % Apply velocity limits
            v = max(-obj.maxLinearVelocity, min(v, obj.maxLinearVelocity));
            omega = max(-obj.maxAngularVelocity, min(omega, obj.maxAngularVelocity));
            
            % Scale velocity based on distance to target
            if distanceToFinish < obj.goalThreshold * 2
                v = v * (distanceToFinish / (obj.goalThreshold * 2));
            end
        end
        
        % send velocities to controller
        function sendVelocityCommand(obj, v, omega)
            msg = ros2message('geometry_msgs/Twist');
            msg.linear.x = v;
            msg.angular.z = omega;
            send(obj.velPub, msg);
        end
        
        function obj = resetPath(obj)
            % Reset the waypoint index when starting a new path
            obj.currentWaypointIndex = 1;
        end
    end
end