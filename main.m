% Load occupancy grid from YAML/PNG pair
yamlFilename = 'carter_warehouse_navigation.yaml';
yamlData = readYAMLFile(yamlFilename); % You'll need to implement this

% Load the PNG
pngFilename = yamlData.image; % The YAML typically specifies the PNG filename
image = imread(pngFilename);

% Convert image to occupancy grid
grayImage = rgb2gray(image);
bwMap = grayImage < (yamlData.occupied_thresh * 255);  % Threshold

%imshow(bwMap);  % Visualize binary occupancy gr

%title('Binary Occupancy Map');

% Create occupancy map
resolution = yamlData.resolution;
map = occupancyMap(bwMap, 1 / resolution);

% Set origin (shift map to proper position)
origin = yamlData.origin;
%setOccupancy(map, origin(1:2), 0);  % Mark origin as free for visualization

% Visualize
% figure;
% show(map);
% title('Ground Truth Occupancy Map');

%clear explorer  % Delete existing explorer object
%delete(ros2("Node","robot_explorer")); % Clean up any lingering nodes
%delete(ros2("Node","ground_truth"));

node = ros2node('ground_truth');

% Create ROS2 publisher
mapPub = ros2publisher(node, '/map', 'nav_msgs/OccupancyGrid');

% Prepare occupancy grid message
msg = ros2message(mapPub);
msg.header.frame_id = 'map';
msg.info.width = uint32(map.GridSize(1));
msg.info.height = uint32(map.GridSize(2));
msg.info.resolution = single(resolution);
msg.info.origin.position.x = 0;
msg.info.origin.position.y = -24;

% Flatten occupancy data (1D array required for ROS)
msg.data = int8(reshape(occupancyMatrix(map)*100, [], 1));


% Publish the map
send(mapPub, msg);
disp('Published ground truth map to /map');

% Before creating RobotExplorer
disp('RobotExplorer initialization parameters:');
disp(['mapSize: ' num2str(map.GridSize)]);
disp(['resolution: ' num2str(resolution)]);

% Initialize robot explorer
robot = Robot(map);
%explorer.fullMap = fullMap;  % Set ground truth map

% Set goal and start navigation
goal = [10, 33];  % Example goal position
robot.navigate(goal);

disp('Robot is running. Send goals through ROS2 topic /goal_pose');
disp('Press Enter to quit and cleanup timers...');

% Wait for Enter key
input('');
disp('Cleaning up...');
% % Stop all timers
% try
%     if ~isempty(robot.controlTimer) && isvalid(robot.controlTimer)
%     stop(robot.controlTimer);
%     delete(robot.controlTimer);
% 
%     end
% catch e
%     disp(e.message);
% end
% try
%     if ~isempty(robot.poseTimer) && isvalid(robot.poseTimer)
%         stop(robot.poseTimer);
%         delete(robot.poseTimer);
%     end
% catch e
%     disp(e.message);
% end
% try
%     if ~isempty(robot.ekf.timer) && isvalid(robot.ekf.timer)
%         stop(robot.ekf.timer);
%         delete(robot.ekf.timer);
%         delete(robot.ekf.node)
%     end
% catch e
%     disp(e.message);
% end

% Delete the robot object
delete(robot);

delete(node);

disp('Script terminated cleanly.');