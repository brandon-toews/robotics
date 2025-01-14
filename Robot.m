classdef Robot < handle
    properties
        robotMap % internal robot map
        currentPose % current pose
        goal % goal to path plan and navigate to
        currentPath %current calculated path
        node % ros2 node for robot object
        frontLidarSub %ros2 subscriber
        backLidarSub %ros2 subscriber
        stateEstimator % State estimation object
        estimationMode % what type of state estimation
        planner % a star
        controller % differential drive controller send inputs to robot in Isaac sim
        robotMapPub % map publisher to see in RViz2
        currentPosePub % pose publisher RViz2
        % Path publisher
        pathPub % planned path publisher for RViz2
        resolution % resolution of occupancy map
        controlTimer    % Timer for real-time control
        isNavigating   % Flag to track navigation state
        poseTimer % timer to update pose info
        goalSub % goal publisher
    end
    methods
        function obj = Robot(map, estimationMode)
            if nargin < 2
                estimationMode = 'fusion';  % Default to sensor fusion
            end

            obj.estimationMode = estimationMode;
            mapSize = map.GridSize;
            obj.resolution = 1/map.Resolution;
            obj.node = ros2node('robot_explorer');
             % Print input parameters
            disp('Creating RobotExplorer:');
            disp(['Map size (pixels): ' num2str(mapSize)]);
            disp(['Resolution (meters/pixel): ' num2str(obj.resolution)]);
            
            % Calculate map dimensions in meters
            map_width_meters = mapSize(1) * obj.resolution;
            map_height_meters = mapSize(2) * obj.resolution;
            disp(['Map dimensions (meters): ' num2str([map_width_meters, map_height_meters])]);
            
            % Initialize with zeros
            unknownMap = zeros(mapSize(1), mapSize(2));
            

            % Initialize dynamic map (same size and resolution as ground truth)
            obj.robotMap = occupancyMap(unknownMap, 1/obj.resolution);
            

            % Publisher for dynamic map
            obj.robotMapPub = ros2publisher(obj.node, '/updated_map', 'nav_msgs/OccupancyGrid');
            
            % Print actual map dimensions to verify
            [map_height, map_width] = size(obj.robotMap.occupancyMatrix);
            disp(['Actual map dimensions (cells): ' num2str([map_height, map_width])]);
            
            obj.planner = plannerAStarGrid(obj.robotMap);
            obj.controller = DiffDrivePathController(obj.node);
            timeStep = 0.3;
            obj.stateEstimator = StateEstimator(estimationMode, 5.975, 16.975, timeStep);
            %obj.stateEstimator.start();
            
            % Get current state
            obj.currentPose = obj.stateEstimator.getState();

            % current pose publisher for displaying in RViz2
            obj.currentPosePub = ros2publisher(obj.node, '/robot_pose', 'geometry_msgs/PoseStamped');
            
            % Lidar Subscriber
            obj.frontLidarSub = ros2subscriber(obj.node, '/front_2d_lidar/scan', 'sensor_msgs/LaserScan');
            %obj.backLidarSub = ros2subscriber(obj.node, '/back_2d_lidar/scan', 'sensor_msgs/LaserScan');

            % Subscribe to pose updates from sensor fusion
            addlistener(obj.stateEstimator, 'PoseUpdated', @(~, ~)obj.updateMap());

            obj.goalSub = ros2subscriber(obj.node, '/goal_pose', 'geometry_msgs/PoseStamped', @obj.goalCallback);
            
            % Path publisher
            obj.pathPub = ros2publisher(obj.node, '/planned_path', 'nav_msgs/Path');

            % Clean up any existing timers
            %delete(timerfindall);

            % Create control timer (50 Hz)
            obj.controlTimer = timer('ExecutionMode', 'fixedRate', ...
                                   'Period', 0.02, ...  % 50 Hz
                                   'TimerFcn', @(~,~)obj.controlLoop());
            obj.isNavigating = false;

           
            obj.poseTimer = timer('ExecutionMode', 'fixedRate', ...
                                 'Period', 1.0, ...
                                 'TimerFcn', @(~,~)obj.publishPose());
            start(obj.poseTimer);

            
    
        end
        function publishPose(obj)
            if isempty(obj.currentPose)
                return;
            end
            try
                % Create and populate PoseStamped message
                poseMsg = ros2message('geometry_msgs/PoseStamped');
                
                % Set header
                poseMsg.header.frame_id = 'map';
                poseMsg.header.stamp = ros2time(obj.node,'now');
               
                % Set position
                poseMsg.pose.position.x = (obj.robotMap.GridSize(1)*obj.resolution)-obj.currentPose(2);
                poseMsg.pose.position.y = obj.currentPose(1);
                poseMsg.pose.position.z = 0;
                
                % Convert theta to quaternion (rotate by -pi/2 to align with ROS convention)
                quat = eul2quat([obj.currentPose(3) + pi/2, 0, 0]);
                poseMsg.pose.orientation.w = quat(1);
                poseMsg.pose.orientation.x = quat(2);
                poseMsg.pose.orientation.y = quat(3);
                poseMsg.pose.orientation.z = quat(4);
                  
                % Publish pose
                send(obj.currentPosePub, poseMsg);
            catch e
                disp(['Error message: ', e.message]);
            end
        end
        function goalCallback(obj, msg)
            newGoal = [msg.pose.position.y, (obj.robotMap.GridSize(1)*obj.resolution)-msg.pose.position.x];
            disp('Selected Nav Goal:')
            disp(newGoal);
            % Stop current navigation if running
            if obj.isNavigating
                obj.isNavigating = false;
                % Stop the robot until we calculate new path
                obj.controller.sendVelocityCommand(0, 0);
                stop(obj.controlTimer);
                obj.stateEstimator.trajectoryAnalysis(obj.currentPath);
            end
            % Start navigation to new goal
            obj.navigate(newGoal);
        end
        % Insert lidar scan into map to update with obstacles
        function insertScans2Map(obj, scans, isFront)
            if ~isempty(scans)
                % Get current occupancy values
                currentOccGrid = occupancyMatrix(obj.robotMap);
                newMap = obj.robotMap.copy();
                % Clean up ranges - replace invalid values (negative or inf) with max range
                ranges = double(scans.ranges);
                maxRange = double(scans.range_max);
                
                % Replace negative values and inf with max range
                ranges(ranges < 0.2 | isinf(ranges)) = maxRange;
                
                % Generate angle array
                angles = linspace(scans.angle_min, scans.angle_max, length(ranges));
                
                % Create lidarScan object with cleaned ranges
                cleanedScan = lidarScan(ranges, angles);
    
                
                robotX = obj.currentPose(1);
                robotY = obj.currentPose(2);
                robotTheta = obj.currentPose(3);

                %disp('map origin')
                %disp(mapOrigin)
                globalPose = [robotX, robotY, robotTheta];  % Transform to map frame
    
                %disp('Global Pose')
                %disp(globalPose)
                
                if isFront
                    % Transform scan to match coordinate system
                    transformedScan = transformScan(cleanedScan, [0, 0, pi]); % Rotate 180°
                else
                    transformedScan = transformScan(cleanedScan, [0, 0, 0]);
                end
    
                try
                    % Insert LIDAR data using the transformed scan
                    insertRay(newMap, globalPose, transformedScan, maxRange);
                    newOccGrid = occupancyMatrix(newMap);

                    % Merge front scan with current map
                    currentOccGrid = max(currentOccGrid, newOccGrid);
                    % Update the actual map
                    setOccupancy(obj.robotMap, currentOccGrid);
                catch e
                    disp('Error in insertRay:');
                    disp(['Position: [', num2str(globalPose), ']']);
                    disp(['Error message: ', e.message]);
                end
            end
        end
        function updateMap(obj)
            % Lidar insertion callback triggered by pose update
            obj.currentPose = obj.stateEstimator.getState();
            %backMsg = receive(obj.backLidarSub, 1);
            try
                frontMsg = receive(obj.frontLidarSub, 0.05); % Grab latest lidar scan
                if ~isempty(frontMsg)
                    % Get current state from EKF using getState()
                    %obj.currentPose = obj.ekf.getState();
                    obj.insertScans2Map(frontMsg, true);
                    %obj.insertScans2Map(backMsg, false);
        
                    testmsg = ros2message('nav_msgs/OccupancyGrid');
                    testmsg.header.frame_id = 'map';
                    testmsg.info.width = uint32(obj.robotMap.GridSize(1));
                    testmsg.info.height = uint32(obj.robotMap.GridSize(2));
                    testmsg.info.resolution = single(1/obj.robotMap.Resolution);
                    
                    % Flatten occupancy data (1D array required for ROS)
                    testmsg.data = int8(reshape(occupancyMatrix(obj.robotMap)*100, [], 1));
                    
                    % Publish the map
                    send(obj.robotMapPub, testmsg);
                end
            catch e
                 %disp(e.message);
            end
        end

        % Add this method to your Robot class
        function savePath(~, path)
            % Save planned path to CSV
            filename = 'planned_path.csv';
            fileID = fopen(filename, 'w');
            fprintf(fileID, 'x,y\n');  % Header
            for i = 1:size(path, 1)
                fprintf(fileID, '%f,%f\n', path(i,1), path(i,2));
            end
            fclose(fileID);
        end
        
        function obj = navigate(obj, goal)
            obj.goal = goal;
            disp('Planning initial path...');
            obj.currentPath = obj.planPath();

            % Save the planned path
            obj.savePath(obj.currentPath);

            disp(['Path has ', num2str(size(obj.currentPath, 1)), ' waypoints']);
            obj.controller.resetPath();
            
            % Start real-time control
            obj.isNavigating = true;

            obj.stateEstimator.startLogging();
            start(obj.controlTimer);
           
        end
        
        function controlLoop(obj)
            try
                if obj.reachedGoal()
                    disp('Reached Goal!');
                    obj.isNavigating = false;
                    obj.controller.sendVelocityCommand(0, 0);
                    stop(obj.controlTimer);
                    % Analyze trajectory with current estimation mode
                    obj.stateEstimator.trajectoryAnalysis(obj.currentPath);
                    return
                end
                
                if obj.needsReplanning()
                    disp('Replanning path...');
                    obj.currentPath = obj.planPath();
                end
                
                % Get current state from estimator
                obj.currentPose = obj.stateEstimator.getState();
                
                % Compute and send velocity commands
                [v, omega] = obj.controller.computeVelocityCommands(obj.currentPose, obj.currentPath);
                obj.controller.sendVelocityCommand(v, omega);
                
                % Update velocity command for dead reckoning if needed
                if strcmp(obj.estimationMode, 'DeadReckoning')
                    obj.stateEstimator.updateVelocityCommand(v, omega);
                end
                
            catch e
                disp('Error in control loop:');
                disp(e.message);
                obj.isNavigating = false;
                obj.controller.sendVelocityCommand(0, 0);
            end
        end
        
        function cells = discretizePath(obj, path)
            % Convert continuous path points to discrete map cells
            % Ensures we check all cells the path passes through
            
            cells = [];
            for i = 1:size(path,1)-1
                % Get points for current path segment
                start = path(i,:);
                finish = path(i+1,:);
                % disp(start);
                % disp(finish);
                
                % Use raycast for cell interpolation
                segmentCells = raycast(obj.robotMap, start, finish);

                cells = [cells; segmentCells];
            end
            
            % Remove duplicates
            cells = unique(cells, 'rows');
        end

        
        function path = planPath(obj)

            %mapOrigin = obj.robotMap.GridLocationInWorld;
            %disp(['Map origin in world coordinates: ', num2str(mapOrigin)]);
            
            % Debug prints
            disp('Current pose format:');
            disp(obj.currentPose);
            disp('Goal format:');
            disp(obj.goal);
            
            % Ensure we have 1x2 vectors for world2grid
            startPos = [obj.currentPose(1), obj.currentPose(2)];  % Extract x,y as 1x2
            goalPos = [obj.goal(1), obj.goal(2)];  % Extract x,y as 1x2

            % Add after your current debug prints
            %obj.debugGridConversion('Start', startPos(1), startPos(2));
            %obj.debugGridConversion('Goal', goalPos(1), goalPos(2));

            disp('Start position for world2grid:');
            disp(startPos);
            disp('Goal position for world2grid:');
            disp(goalPos);
            
            % Convert to grid coordinates
            tic;
            start = obj.robotMap.world2grid(startPos);
            convTime = toc;
            disp(['World2grid conversion took: ', num2str(convTime), ' seconds']);
            finish = obj.robotMap.world2grid(goalPos);

            disp('Start position in grid postion:');
            disp(start);
            disp('Goal position in grid postion:');
            disp(finish);
            
            
            % Get path from planner
            gridPath = plan(obj.planner, start, finish);
            
            % Convert grid path to world coordinates
            worldPath = obj.robotMap.grid2world(gridPath);

            % Print detailed path information
            % disp('Planned path in world coordinates:');
            % disp('Format: [x, y]');
            % for i = 1:size(worldPath, 1)
            %     disp(['Point ', num2str(i), ': [', num2str(worldPath(i,1), '%.2f'), ', ', num2str(worldPath(i,2), '%.2f'), ']']);
            % end
            % show(obj.planner)
            convert2Grid = obj.robotMap.GridSize(1)*obj.resolution;
            % Create Path message
            pathMsg = ros2message('nav_msgs/Path');
            pathMsg.header.frame_id = 'map';
            pathMsg.header.stamp = ros2time(obj.node,'now');
            
            % Populate poses array
            for i = 1:size(worldPath, 1)
                pose = ros2message('geometry_msgs/PoseStamped');
                pose.header.frame_id = 'map';
                pose.header.stamp = ros2time(obj.node,'now');
                
                % Set position
                pose.pose.position.x = convert2Grid - worldPath(i, 2);
                pose.pose.position.y = worldPath(i, 1);
                pose.pose.position.z = 0.0;
                
                pathMsg.poses(i) = pose;
            end
            
            % Publish path
            send(obj.pathPub, pathMsg);
            
            % Return the path in world coordinates
            path = worldPath;
        end
        function replan = needsReplanning(obj)
            if isempty(obj.currentPath)
                replan = true;
                return;
            end
            pathCells = obj.discretizePath(obj.currentPath);
            % inflatedMap = obj.robotMap.copy();
            % inflate(inflatedMap,1)
            % disp('Path cells:');
            % disp(pathCells);
            occupiedCells = getOccupancy(obj.robotMap, pathCells, "grid") > 0.65;
            % disp('Occupied cells:');
            % disp(occupiedCells);
            replan = any(occupiedCells);
            %disp(replan)
        end
        function reached = reachedGoal(obj)
            if isempty(obj.currentPose) || isempty(obj.goal)
                reached = false;
                return;
            end
            %disp('current pos:');
            %disp(obj.currentPose(1:2)');
            %disp('Goal position');
            %disp(obj.goal);
            dist = norm(obj.currentPose(1:2)' - obj.goal);
            %disp('Distance');
            %disp(dist);
            reached = dist < 0.2;
        end
        
        % Delete method for cleaning up timers and ros2 stuff
        function delete(obj)
            disp('Cleaning up Robot object...');

            try
                if ~isempty(obj.controlTimer) && isvalid(obj.controlTimer)
                stop(obj.controlTimer);
                delete(obj.controlTimer);
            
                end
            catch e
                disp(e.message);
            end
            try
                if ~isempty(obj.poseTimer) && isvalid(obj.poseTimer)
                    stop(obj.poseTimer);
                    delete(obj.poseTimer);
                end
            catch e
                disp(e.message);
            end
            try
                if ~isempty(obj.stateEstimator.timer) && isvalid(obj.stateEstimator.timer)
                    stop(obj.stateEstimator.timer);
                    delete(obj.stateEstimator.timer);
                end
            catch e
                disp(e.message);
            end
            
            % Clean up subscribers
            if ~isempty(obj.frontLidarSub)
                clear obj.frontLidarSub;
            end
            if ~isempty(obj.backLidarSub)
                clear obj.backLidarSub;
            end
            if ~isempty(obj.goalSub)
                clear obj.goalSub;
            end
            
            % Clean up publishers
            if ~isempty(obj.robotMapPub)
                clear obj.robotMapPub;
            end
            if ~isempty(obj.currentPosePub)
                clear obj.currentPosePub;
            end
            if ~isempty(obj.pathPub)
                clear obj.pathPub;
            end
            
            % Clean up node last
            if ~isempty(obj.node)
                delete(obj.node);
            end
            
            disp('Robot cleanup complete.');
        end
    end
end