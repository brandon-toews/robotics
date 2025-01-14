classdef StateEstimator < handle
    properties
        mode            % 'Fusion', 'odom_only', or 'DeadReckoning'
        x               % State vector [x, y, theta, v, omega]
        P               % Covariance matrix (for EKF mode)
        Q               % Process noise
        R_odom % Odom Measurement noise - Jacobian
        R_imu % IMU Measurement noise - Jacobian
        odomMeasurementNoise % Odom Measurement noise - Column Vector
        imuMeasurementNoise % IMU Measurement noise - Column Vector
        dt             % Time step
        imuSub         % IMU subscriber
        odomSub        % Odometry subscriber
        groundTruthSub % Ground truth subscriber
        node           % ROS2 Node
        logFile        % File handle for logging
        isLogging = false
        T_world_odom   % Transform from odom to world frame
        timer
        
        % Parameters for dead reckoning
        last_cmd_vel   % Last received velocity command
        last_update_time % Time of last update
    end
    
    events
        PoseUpdated    % Event to trigger map updates
    end
    
    methods
        function obj = StateEstimator(mode, initialX, initialY, dt)
            obj.mode = mode;
            obj.dt = dt;
            
            % Initialize transform
            obj.T_world_odom = [1, 0, initialX;
                               0, 1, initialY;
                               0, 0, 1];
            
            % Create ROS2 node
            obj.node = ros2node(['state_estimator_' mode]);
            
            % Initialize state
            obj.x = [0; 0; 0; 0; 0];

            % Measurement noise (odom and IMU)
            obj.odomMeasurementNoise = [0.05; % x - odom
                                        0.05; % y - odom
                                        0.03; % theta - odom
                                        0.1; % v - odom
                                        0.05]; % omega - IMU
            
            % Measurement noise (odom and IMU)
            obj.imuMeasurementNoise = [0.05; % v - IMU
                                       0.05]; % omega - IMU


            % Initialize covariance and noise matrices for EKF mode
            if strcmp(mode, 'Fusion')
                obj.P = eye(5);
                obj.Q = diag([0.008, 0.008, 0.053, 0.0005, 0.05]);
                % Noise measurments converted to Jacobian
                obj.R_odom = diag(obj.odomMeasurementNoise');
                obj.R_imu = diag(obj.imuMeasurementNoise');
            end
            
            % Set up subscribers based on mode
            if strcmp(mode, 'Fusion') || strcmp(mode, 'OdomOnly')
                obj.odomSub = ros2subscriber(obj.node, '/chassis/odom', 'nav_msgs/Odometry');
            end
            
            if strcmp(mode, 'Fusion')
                obj.imuSub = ros2subscriber(obj.node, '/chassis/imu', 'sensor_msgs/Imu');
            end
            
            
            % Ground truth subscriber for all modes
            obj.groundTruthSub = ros2subscriber(obj.node, '/ground_truth_pose', 'tf2_msgs/TFMessage');

            % Create timer
            obj.timer = timer('ExecutionMode', 'fixedRate', ...
                            'Period', dt, ...
                            'TimerFcn', @(~,~)obj.updateState());
            start(obj.timer);
        end

        function trajectoryAnalysis(obj, plannedPath)
            % Modified to handle different estimation modes
            obj.stopLogging();
            
            % Read the data for current mode
            data = readmatrix(['trajectory_log_' obj.mode '.csv']);

            % Debug current state conversion
            current_local = [obj.x(1); obj.x(2); 1];
            current_world = obj.T_world_odom * current_local;
            disp('Current state conversion:');
            disp(['Local: [', num2str(obj.x(1)), ', ', num2str(obj.x(2)), ']']);
            disp(['World: [', num2str(current_world(1)), ', ', num2str(current_world(2)), ']']);
            
            % Debug first logged point conversion
            first_local = [data(1,1); data(1,2); 1];
            first_world = obj.T_world_odom * first_local;
            disp('First logged point conversion:');
            disp(['Local: [', num2str(data(1,1)), ', ', num2str(data(1,2)), ']']);
            disp(['World: [', num2str(first_world(1)), ', ', num2str(first_world(2)), ']']);
            
            % Debug transformation matrix
            disp('Transformation matrix:');
            disp([obj.T_world_odom]);
            
            % Transform odometry readings one by one for better debugging
            world_state_estimate = zeros(size(data, 1), 2);
            world_ground = zeros(size(data, 1), 2);
            for i = 1:size(data, 1)
                % Get odometry position in local frame
                local_pos = [data(i, 1); data(i, 2); 1];
                % Transform to world frame
                world_pos = obj.T_world_odom * local_pos;
                world_state_estimate(i, :) = world_pos(1:2)';
                world_ground(i, :) = [data(i, 4)+11.975, data(i, 5)+17.975];
            end

            if nargin > 1 && ~isempty(plannedPath)
                % Analyze path deviation
                % For each point in the ground truth, find closest point on planned path
                path_errors = zeros(size(world_ground, 1), 1);
                for i = 1:size(world_ground, 1)
                    % Calculate distances to all points on planned path
                    distances = sqrt(sum((plannedPath - world_ground(i,:)).^2, 2));
                    % Find minimum distance (closest point)
                    path_errors(i) = min(distances);
                end
                
                % Calculate path error statistics
                mean_path_error = mean(path_errors);
                median_path_error = median(path_errors);
                max_path_error = max(path_errors);
                std_path_error = std(path_errors);
                rms_path_error = rms(path_errors);
                
                % Create error distribution subplot
                figure('Name', 'Path Error Analysis');
                subplot(2,1,1);
                histogram(path_errors, 30, 'Normalization', 'probability');
                title([obj.mode, ' - Path Error Distribution']);
                xlabel('Error Distance (m)');
                ylabel('Probability');
                grid on;
                
                % Plot error over time
                subplot(2,1,2);
                plot(1:length(path_errors), path_errors, 'b-');
                title([obj.mode, ' - Path Error Over Time']);
                xlabel('Sample Number');
                ylabel('Error Distance (m)');
                grid on;
                
                % Print statistics
                disp('Path Error Statistics:');
                disp(['Mean Error: ', num2str(mean_path_error), ' m']);
                disp(['Median Error: ', num2str(median_path_error), ' m']);
                disp(['Max Error: ', num2str(max_path_error), ' m']);
                disp(['Standard Deviation: ', num2str(std_path_error), ' m']);
                disp(['RMS Error: ', num2str(rms_path_error), ' m']);
                
                % Calculate what percentage of time robot was within different error thresholds
                error_thresholds = [0.1, 0.2, 0.5, 1.0]; % meters
                for threshold = error_thresholds
                    within_threshold = sum(path_errors < threshold) / length(path_errors) * 100;
                    disp(['Time within ', num2str(threshold), 'm: ', num2str(within_threshold), '%']);
                end
            end
            
            % Create plot
            figure('Name', 'Robot Trajectory Analysis');
            hold on;
            grid on;
            
            % Plot trajectories
            plot(world_state_estimate(:,1), world_state_estimate(:,2), 'b-', 'DisplayName', 'State Estimation');
            plot(world_ground(:,1), world_ground(:,2), 'r-', 'DisplayName', 'Ground Truth');
            
            % Plot planned path if provided
            if nargin > 1 && ~isempty(plannedPath)
                plot(plannedPath(:,1), plannedPath(:,2), 'g--', 'LineWidth', 2, 'DisplayName', 'Planned Path');
                % Plot start and goal of planned path
                plot(plannedPath(1,1), plannedPath(1,2), 'go', 'MarkerSize', 10, 'DisplayName', 'Path Start');
                plot(plannedPath(end,1), plannedPath(end,2), 'g*', 'MarkerSize', 10, 'DisplayName', 'Path Goal');
            end
            
            % Plot start points
            plot(world_state_estimate(1,1), world_state_estimate(1,2), 'bo', 'DisplayName', 'State Estimate Start', 'MarkerSize', 10);
            plot(world_ground(1,1), world_ground(1,2), 'ro', 'DisplayName', 'Truth Start', 'MarkerSize', 10);
            
            % Add labels and legend
            title([obj.mode, ' - Robot Trajectory Comparison']);
            xlabel('X Position (m)');
            ylabel('Y Position (m)');
            legend('Location', 'best');
            axis equal;
            
            % Print statistics
            disp('Trajectory Statistics:');
            total_distance_odom = sum(sqrt(sum(diff(world_state_estimate).^2, 2)));
            total_distance_truth = sum(sqrt(sum(diff(world_ground).^2, 2)));
            disp(['Total Distance (State Estimate): ', num2str(total_distance_odom), ' m']);
            disp(['Total Distance (Ground Truth): ', num2str(total_distance_truth), ' m']);
            if nargin > 1 && ~isempty(plannedPath)
                planned_distance = sum(sqrt(sum(diff(plannedPath).^2, 2)));
                disp(['Planned Path Length: ', num2str(planned_distance), ' m']);
            end
            
            % Calculate errors
            x_errors = world_state_estimate(:,1) - world_ground(:,1);
            y_errors = world_state_estimate(:,2) - world_ground(:,2);
            theta_errors = data(:,3) - data(:,6);  % angular errors
            
            % Create figure with subplots
            figure('Name', 'Trajectory Error Analysis');
            
            % X-error distribution
            subplot(3,2,1);
            histogram(x_errors, 30, 'Normalization', 'probability');
            title([obj.mode, ' - X Position Error Distribution']);
            xlabel('Error (m)');
            ylabel('Probability');
            
            % Y-error distribution
            subplot(3,2,2);
            histogram(y_errors, 30, 'Normalization', 'probability');
            title([obj.mode, ' - Y Position Error Distribution']);
            xlabel('Error (m)');
            ylabel('Probability');
            
            % Theta error distribution
            subplot(3,2,3);
            histogram(theta_errors, 30, 'Normalization', 'probability');
            title([obj.mode, ' - Theta Error Distribution']);
            xlabel('Error (rad)');
            ylabel('Probability');
            
            % 2D error scatter
            subplot(3,2,4);
            scatter(x_errors, y_errors, 10, 'filled');
            title([obj.mode, ' - Position Error Distribution']);
            xlabel('X Error (m)');
            ylabel('Y Error (m)');
            axis equal;
            grid on;
            
            % Error over time
            subplot(3,2,[5,6]);
            plot(1:length(x_errors), sqrt(x_errors.^2 + y_errors.^2));
            title([obj.mode, ' - Total Position Error Over Time']);
            xlabel('Sample Number');
            ylabel('Error Magnitude (m)');
            grid on;
            
            % Print statistics
            disp([obj.mode, ' - Error Statistics:']);
            disp('X Position Error:');
            disp(['  Mean: ', num2str(mean(x_errors))]);
            disp(['  Median: ', num2str(median(x_errors))]);
            disp(['  Std Dev: ', num2str(std(x_errors))]);
            disp(['  RMS: ', num2str(rms(x_errors))]);
            
            disp('Y Position Error:');
            disp(['  Mean: ', num2str(mean(y_errors))]);
            disp(['  Median: ', num2str(median(y_errors))]);
            disp(['  Std Dev: ', num2str(std(y_errors))]);
            disp(['  RMS: ', num2str(rms(y_errors))]);
            
            disp('Theta Error (rad):');
            disp(['  Mean: ', num2str(mean(theta_errors))]);
            disp(['  Median: ', num2str(median(theta_errors))]);
            disp(['  Std Dev: ', num2str(std(theta_errors))]);
            disp(['  RMS: ', num2str(rms(theta_errors))]);
            
            % Calculate total position error statistics
            total_pos_error = sqrt(x_errors.^2 + y_errors.^2);
            disp([obj.mode, ' - Total Position Error:']);
            disp(['  Mean: ', num2str(mean(total_pos_error))]);
            disp(['  Median: ', num2str(median(total_pos_error))]);
            disp(['  Std Dev: ', num2str(std(total_pos_error))]);
            disp(['  RMS: ', num2str(rms(total_pos_error))]);
        end
        
        function startLogging(obj)
            % Create unique log file for each mode
            filename = ['trajectory_log_' obj.mode '.csv'];
            obj.logFile = fopen(filename, 'w');
            fprintf(obj.logFile, 'est_x,est_y,est_theta,truth_x,truth_y,truth_theta\n');
            obj.isLogging = true;
        end
        
        function stopLogging(obj)
            if obj.isLogging
                fclose(obj.logFile);
                obj.isLogging = false;
            end
        end
        
        function x_next = odomUpdate(obj, odomMsg)
            % Update state using only odometry measurements
            x_next = obj.x;
            x_next(1) = odomMsg.pose.pose.position.x + randn() * obj.odomMeasurementNoise(1);
            x_next(2) = odomMsg.pose.pose.position.y + randn() * obj.odomMeasurementNoise(2);
            quat = [odomMsg.pose.pose.orientation.w, ...
                    odomMsg.pose.pose.orientation.x, ...
                    odomMsg.pose.pose.orientation.y, ...
                    odomMsg.pose.pose.orientation.z];
            euler = quat2eul(quat, 'ZYX');
            x_next(3) = euler(1) + randn() * obj.odomMeasurementNoise(3);
            x_next(4) = sqrt(odomMsg.twist.twist.linear.x^2 + odomMsg.twist.twist.linear.y^2);
            x_next(4) = x_next(4) + randn() * obj.odomMeasurementNoise(4);
            x_next(5) = odomMsg.twist.twist.angular.z + randn() * obj.odomMeasurementNoise(5);
        end

        function ekfUpdate(obj, odomMsg, imuMsg)
            % Extract odometry and IMU data
                    odom_x = odomMsg.pose.pose.position.x;
                    odom_y = odomMsg.pose.pose.position.y;
                    odom_theta = quat2eul([odomMsg.pose.pose.orientation.w, ...
                                  odomMsg.pose.pose.orientation.x, ...
                                  odomMsg.pose.pose.orientation.y, ...
                                  odomMsg.pose.pose.orientation.z], 'ZYX');
                    odom_theta = odom_theta(1);
                    odom_lin_vel = sqrt(odomMsg.twist.twist.linear.x^2 + odomMsg.twist.twist.linear.y^2);
                    odom_ang_vel = odomMsg.twist.twist.angular.z;
                    
                    imu_lin_acc = imuMsg.linear_acceleration.x;
                    imu_ang_vel = imuMsg.angular_velocity.z;
                    
                    % Prediction step
                    F = obj.compute_state_to_jacobian();
                    obj.x = obj.state_transition();
                    obj.P = F * obj.P * F' + obj.Q;
                    % Odometry Update
                    H_odom = eye(5);
                    z_odom = [odom_x; 
                         odom_y; 
                         odom_theta;
                         odom_lin_vel;
                         odom_ang_vel] + diag(obj.R_odom) .* randn(5, 1); % Simulated noisy odom
 
                    % Calculate Kalman Gain for Odom
                    K = obj.P * H_odom' / (H_odom * obj.P * H_odom' + obj.R_odom);
                    obj.x = obj.x + K * (z_odom - H_odom * obj.x);
                    obj.P = (eye(5) - K * H_odom) * obj.P;

                    v_est = obj.x(4) + imu_lin_acc * obj.dt;
                    
                    % IMU Update
                    H_imu = [0, 0, 0, 1, 0;
                             0, 0, 0, 0, 1];
                    z_imu = [v_est; 
                         imu_ang_vel] + diag(obj.R_imu) .* randn(2,1); % Simulated noisy IMU


                    % Calculate Kalman Gain for Odom
                    K = obj.P * H_imu' / (H_imu * obj.P * H_imu' + obj.R_imu);
                    obj.x = obj.x + K * (z_imu - H_imu * obj.x);
                    obj.P = (eye(5) - K * H_imu) * obj.P;
                    
                  
        end
        
        function updateState(obj)
            try
                
                switch obj.mode
                    case 'Fusion'
                        odomMsg = receive(obj.odomSub, 1);
                        imuMsg = receive(obj.imuSub, 1);
                        
                        if ~isempty(odomMsg) && ~isempty(imuMsg)
                            % EKF prediction and update steps
                            obj.ekfUpdate(odomMsg, imuMsg);
                        end
                        
                    case 'OdomOnly'
                        odomMsg = receive(obj.odomSub, 1);
                        if ~isempty(odomMsg)
                            % Only Odom
                            obj.x = obj.odomUpdate(odomMsg);
                        end
                    
                    % Just dead reckoning
                    case 'DeadReckoning'
                        obj.x = obj.state_transition();
                       
                end

                % Get ground truth for logging
                if obj.isLogging
                    groundTruthMsg = receive(obj.groundTruthSub, 1);
                    if ~isempty(groundTruthMsg)
                        truth_x = groundTruthMsg.transforms.transform.translation.x;
                        truth_y = groundTruthMsg.transforms.transform.translation.y;
                        truth_quat = [groundTruthMsg.transforms.transform.rotation.w, ...
                                    groundTruthMsg.transforms.transform.rotation.x, ...
                                    groundTruthMsg.transforms.transform.rotation.y, ...
                                    groundTruthMsg.transforms.transform.rotation.z];
                        truth_euler = quat2eul(truth_quat, 'ZYX');
                        truth_theta = truth_euler(1);

                        % Log raw odometry values
                        fprintf(obj.logFile, '%f,%f,%f,%f,%f,%f\n', ...
                            obj.x(1), obj.x(2), obj.x(3), ...
                            truth_x, truth_y, truth_theta);
                    end
                end
                
                
                % Notify listeners
                notify(obj, 'PoseUpdated');
                
            catch e
                disp(['Error in state update (' obj.mode '): ' e.message]);
            end
        end
        
        function state = getState(obj)
            if isempty(obj.x)
                state = [];
            else
                % Transform local state to world coordinates
                local_pos = [obj.x(1); obj.x(2); 1];
                world_pos = obj.T_world_odom * local_pos;
                
                state = obj.x;
                state(1) = world_pos(1);
                state(2) = world_pos(2);
            end
        end
        
        % Compute Jacobian of the state transition model
        function F = compute_state_to_jacobian(obj)
            
            theta = obj.x(3);
            v = obj.x(4);
            
            F = [1, 0, -v * obj.dt * sin(theta), obj.dt * cos(theta), 0;
                 0, 1,  v * obj.dt * cos(theta), obj.dt * sin(theta), 0;
                 0, 0, 1,                         0,                  obj.dt;
                 0, 0, 0,                         1,                  0;
                 0, 0, 0,                         0,                  1];
        end

        % State transition function
        function x_next = state_transition(obj)
            
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
        
        % update for dead reckoning
        function obj = updateVelocityCommand(obj, v, omega)
            obj.x(4) = v;
            obj.x(5) = omega;
               
        end
        
        % Delete method for cleaning up timers and ros2 stuff
        function delete(obj)
            obj.stopLogging();
            
            % Clean up subscribers
            if ~isempty(obj.imuSub)
                clear obj.imuSub;
            end
            if ~isempty(obj.odomSub)
                clear obj.odomSub;
            end
            if ~isempty(obj.groundTruthSub)
                clear obj.groundTruthSub;
            end
            
            % Clean up node
            if ~isempty(obj.node)
                delete(obj.node);
            end
        end
    end
end