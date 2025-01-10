classdef SensorFusionEKF < handle
    properties
        x  % State [x, y, theta, v, omega]
        P  % Covariance matrix
        Q  % Process noise - Jacobian
        R  % Measurement noise - Jacobian
        measurementNoise % Measurement noise - Column Vector
        dt % Time step
        imuSub  % IMU subscriber
        odomSub % Odometry subscriber
        node    % ROS2 Node
        stateLog  % Logging for trajectory
        running % Boolean for starting and stopping sensor fusion
        timer

        T_world_odom  % Transform from odom to world frame

        x_offset
        y_offset
    end
    events
        PoseUpdated  % Event to trigger lidar callback
    end
    
    methods
        function obj = SensorFusionEKF(initialX, initialY, dt)
            % Constructor

            % Time step
            obj.dt = dt;

            % Store offsets
            obj.x_offset = initialX;
            obj.y_offset = initialY;

            % Store the transform from odom to world
            obj.T_world_odom = [1, 0, initialX;
                                0, 1, initialY;
                                0, 0, 1];

            % Create ros2 node for sensor fusion
            obj.node = ros2node('sensorFusionNode');

            % Initial state [x, y, theta, v, omega]
            obj.x = [0; 0; 0; 0; 0];

            % Debug print
            % disp('EKF Initial State:');
            % disp(obj.x);

            % Initial covariance matrix
            obj.P = eye(5);

            % Process noise covariance
            obj.Q = diag([0.1, 0.1, 0.05, 0.1, 0.05]);

            % Measurement noise (odom and IMU)
            obj.measurementNoise = [0.05; % x - odom
                                    0.05; % y - odom
                                    0.02; % theta - odom
                                    0.1; % v - IMU
                                    0.05]; % omega - IMU

            % Noise measurments converted to Jacobian
            obj.R = diag(obj.measurementNoise');

            % Initialize bool
            obj.running = false;
            
            % ROS2 Subscribers
            obj.imuSub = ros2subscriber(obj.node, '/chassis/imu', 'sensor_msgs/Imu');
            obj.odomSub = ros2subscriber(obj.node, '/chassis/odom', 'nav_msgs/Odometry');
            %start(obj);
            % Create timer
            obj.timer = timer('ExecutionMode', 'fixedRate', ...
                            'Period', dt, ...
                            'TimerFcn', @(~,~)obj.run());
        end

        function obj = start(obj)
            obj.running = true;
            start(obj.timer);
        end

        function obj = stop(obj)
            obj.running = false;
            stop(obj.timer);
            delete(obj.timer);
        end
        
        function x_next = state_transition(obj)
            % State transition function
            theta = obj.x(3);
            % v and omega remain the same
            v = obj.x(4);
            omega = obj.x(5);
            
            % Update state
            x_next = obj.x;
            x_next(1) = obj.x(1) + v * obj.dt * cos(theta);
            x_next(2) = obj.x(2) + v * obj.dt * sin(theta);
            x_next(3) = obj.x(3) + omega * obj.dt;
        end

        % Replace getWorldState() with this:
        function world_state = getState(obj)
            if isempty(obj.x)
                world_state = [];
            else
                % Transform local state to world coordinates when needed
                local_pos = [obj.x(1); obj.x(2); 1];
                world_pos = obj.T_world_odom * local_pos;
                
                world_state = obj.x;
                world_state(1) = world_pos(1);
                world_state(2) = world_pos(2);
            end
        end
        
        function F = compute_state_to_jacobian(obj)
            % Compute Jacobian of the state transition model
            theta = obj.x(3);
            v = obj.x(4);
            
            F = [1, 0, -v * obj.dt * sin(theta), obj.dt * cos(theta), 0;
                 0, 1,  v * obj.dt * cos(theta), obj.dt * sin(theta), 0;
                 0, 0, 1,                         0,                  obj.dt;
                 0, 0, 0,                         1,                  0;
                 0, 0, 0,                         0,                  1];
        end
        
        % Method to perform sensor fusion
        function obj = run(obj)
            try
                odomMsg = receive(obj.odomSub, 1);
                imuMsg = receive(obj.imuSub, 1);
                if ~isempty(odomMsg) && ~isempty(imuMsg)
                    % Extract odometry and IMU data
                    odom_x = odomMsg.pose.pose.position.x;
                    odom_y = odomMsg.pose.pose.position.y;
                    odom_theta = quat2eul([odomMsg.pose.pose.orientation.w, ...
                                  odomMsg.pose.pose.orientation.x, ...
                                  odomMsg.pose.pose.orientation.y, ...
                                  odomMsg.pose.pose.orientation.z], 'ZYX');
                    odom_theta = odom_theta(1);
                    
                    imu_lin_acc = imuMsg.linear_acceleration.x;
                    imu_ang_vel = imuMsg.angular_velocity.z;
                    
                    % Prediction step
                    F = obj.compute_state_to_jacobian();
                    obj.x = obj.state_transition();
                    obj.P = F * obj.P * F' + obj.Q;
                    
                    % Measurement and update
                    v_est = obj.x(4) + imu_lin_acc * obj.dt;
                    z = [odom_x; odom_y; odom_theta; v_est; imu_ang_vel] + diag(obj.R) .* randn(5, 1);
                    
                    % Kalman Gain
                    H = eye(5);
                    K = obj.P * H' / (H * obj.P * H' + obj.R);
                    obj.x = obj.x + K * (z - H * obj.x);
                    obj.P = (eye(5) - K * H) * obj.P;
        
                    % Debug prints
                    %disp('EKF Current pose:');
                    %disp(obj.getState);
        
                    % Trigger PoseUpdated event
                    % notify(obj, 'PoseUpdated');
                end
                
            catch e
                disp('Error in EKF run:');
                disp(e.message);
            end
        end
        function delete(obj)
            disp('Cleaning up Sensor Fusion object...');
            
            % Clean up timer
            try
                if ~isempty(obj.timer) && isvalid(obj.timer)
                    stop(obj.timer);
                    delete(obj.timer);
                end
            catch e
                disp(e.message);
            end
            
            % Clean up subscribers
            if ~isempty(obj.imuSub)
                clear obj.imuSub;
            end
            if ~isempty(obj.odomSub)
                clear obj.odomSub;
            end
  
            
            % Clean up node last
            if ~isempty(obj.node)
                delete(obj.node);
            end
            
            disp('Sensor Fusion cleanup complete.');
        end
    end
end
        % % Method to perform sensor fusion
        % function obj = run(obj)
        %     odomMsg = receive(obj.odomSub, 10);
        %     imuMsg = receive(obj.imuSub, 10);
        %     %groundTruthMsg = receive(obj.groundTruthSub, 10);
        % 
        %     % Extract odometry and ground truth data
        %     odom_x = odomMsg.pose.pose.position.x;
        %     odom_y = odomMsg.pose.pose.position.y;
        %     odom_theta = quat2eul([odomMsg.pose.pose.orientation.w, ...
        %                            odomMsg.pose.pose.orientation.x, ...
        %                            odomMsg.pose.pose.orientation.y, ...
        %                            odomMsg.pose.pose.orientation.z], 'ZYX');
        %     odom_theta = odom_theta(1);
        % 
        %     % groundTruthX = groundTruthMsg.transforms.transform.translation.x;
        %     % groundTruthY = groundTruthMsg.transforms.transform.translation.y;
        %     % groundTruthTheta = quat2eul([groundTruthMsg.transforms.transform.rotation.w, ...
        %     %                              groundTruthMsg.transforms.transform.rotation.x, ...
        %     %                              groundTruthMsg.transforms.transform.rotation.y, ...
        %     %                              groundTruthMsg.transforms.transform.rotation.z], 'ZYX');
        %     % groundTruthTheta = groundTruthTheta(1);
        % 
        %     % Log the data
        %     %obj.odomLog = [obj.odomLog; odom_x, odom_y, odom_theta];
        %     %obj.groundTruthLog = [obj.groundTruthLog; groundTruthX, groundTruthY, groundTruthTheta];
        % 
        %     imu_lin_acc = imuMsg.linear_acceleration.x;
        %     imu_ang_vel = imuMsg.angular_velocity.z;
        % 
        % 
        %     % Prediction step
        %     F = obj.compute_state_to_jacobian();
        %     obj.x = obj.state_transition();
        %     obj.P = F * obj.P * F' + obj.Q;
        % 
        %     % Measurement and update
        %     v_est = obj.x(4) + imu_lin_acc * obj.dt;
        %     z = [odom_x; odom_y; odom_theta; v_est; imu_ang_vel] + diag(obj.R) .* randn(5, 1);
        % 
        %     % Kalman Gain
        %     H = eye(5);
        %     K = obj.P * H' / (H * obj.P * H' + obj.R);
        %     obj.x = obj.x + K * (z - H * obj.x);
        %     obj.P = (eye(5) - K * H) * obj.P;
        % 
        %     % Debug prints
        %     %disp('EKF Current pose:');
        %     %disp(obj.getState);
        % 
        %     % Trigger PoseUpdated event
        %     notify(obj, 'PoseUpdated');
        % end
