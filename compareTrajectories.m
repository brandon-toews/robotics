% After all robots complete their tasks visualize for comparisons
compareTheTrajectories('Fusion', 'OdomOnly', 'DeadReckoning');

compareGroundTruth('Fusion', 'OdomOnly', 'DeadReckoning');

function compareTheTrajectories(varargin)
    disp(' ');
    figure('Name', 'Trajectory Comparison');
    hold on;
    grid on;

    % Initialize transform
    T_world_odom = [1, 0, 5.975;
                    0, 1, 16.975;
                    0, 0, 1];

    planned_path = readmatrix('planned_path.csv');
    
    % Plot estimated vs ground truth trajectories
    plot(planned_path(:,1), planned_path(:,2), ['p' '-'], 'DisplayName', 'Planned Path');
    
    colors = {'b', 'r', 'g'};
    for i = 1:length(varargin)
        mode = varargin{i};
        data = readmatrix(['trajectory_log_' mode '.csv']);
        % Transform odometry readings one by one for better debugging
        world_state_estimate = zeros(size(data, 1), 2);
        for n = 1:size(data, 1)
            local_pos = [data(n, 1); data(n, 2); 1];
            % Transform to world frame
            world_pos = T_world_odom * local_pos;
            world_state_estimate(n, :) = world_pos(1:2)';
        end
        
        % Plot estimated vs ground truth trajectories
        plot(world_state_estimate(:,1), world_state_estimate(:,2), [colors{i} '-'], 'DisplayName', [mode ' est']);
        plot(data(:,4)+11.975, data(:,5)+17.975, [colors{i} '--'], 'DisplayName', [mode ' truth']);
        
        % Calculate and display error metrics
        pos_error = sqrt((world_state_estimate(:,1)-(data(:,4)+11.975)).^2 + (world_state_estimate(:,2)-(data(:,5)+17.975)).^2);
        heading_error = abs(wrapToPi(data(:,3) - data(:,6)));
        
        disp(['=== ' mode ' Statistics ===']);
        disp(['Mean Position Error: ' num2str(mean(pos_error)) ' m']);
        disp(['RMS Position Error: ' num2str(rms(pos_error)) ' m']);
        disp(['Mean Heading Error: ' num2str(mean(heading_error)) ' rad']);
        disp(['RMS Heading Error: ' num2str(rms(heading_error)) ' rad']);
        disp('  ');
    end
    
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Trajectory Comparison Across Different Estimation Methods');
    legend('Location', 'best');
    axis equal;
end

function compareGroundTruth(varargin)
    % Create main comparison plot
    figure('Name', 'Ground Truth Path Analysis');
    subplot(2,2,[1,2]); % Use top half for path plot
    hold on;
    grid on;

    % Initialize transform
    T_world_odom = [1, 0, 5.975;
                    0, 1, 16.975;
                    0, 0, 1];

    % Load and plot planned path
    planned_path = readmatrix('planned_path.csv');
    plot(planned_path(:,1), planned_path(:,2), 'k-', 'LineWidth', 2, 'DisplayName', 'Planned Path');
    
    % Colors and markers for different methods
    colors = {'b', 'r', 'g'};
    markers = {'o', 's', '^'};
    
    % Store metrics for comparison
    path_following_stats = struct();
    
    for i = 1:length(varargin)
        mode = varargin{i};
        data = readmatrix(['trajectory_log_' mode '.csv']);
        
        % Get ground truth trajectory with transformation
        ground_truth = [data(:,4)+11.975, data(:,5)+17.975];
        
        % Plot only ground truth trajectory
        plot(ground_truth(:,1), ground_truth(:,2), [colors{i} '-'], 'LineWidth', 1.5, ...
             'DisplayName', [mode ' path']);
        
        % Mark start and end points
        plot(ground_truth(1,1), ground_truth(1,2), [colors{i} markers{i}], ...
             'DisplayName', [mode ' start'], 'MarkerSize', 10);
        plot(ground_truth(end,1), ground_truth(end,2), [colors{i} 'x'], ...
             'DisplayName', [mode ' end'], 'MarkerSize', 10);
        
        % Calculate path following metrics
        % Distance to planned path at each point
        path_errors = zeros(size(ground_truth, 1), 1);
        for j = 1:size(ground_truth, 1)
            distances = sqrt((planned_path(:,1) - ground_truth(j,1)).^2 + ...
                           (planned_path(:,2) - ground_truth(j,2)).^2);
            path_errors(j) = min(distances);
        end
        
        % Calculate path smoothness (using heading changes)
        headings = atan2(diff(ground_truth(:,2)), diff(ground_truth(:,1)));
        heading_changes = abs(diff(headings));
        smoothness = mean(heading_changes);
        
        % Store statistics
        path_following_stats.(mode).mean_error = mean(path_errors);
        path_following_stats.(mode).max_error = max(path_errors);
        path_following_stats.(mode).path_length = sum(sqrt(sum(diff(ground_truth).^2, 2)));
        path_following_stats.(mode).smoothness = smoothness;
        
        % Calculate percentage within different thresholds
        thresholds = [0.1, 0.2, 0.5];
        for t = 1:length(thresholds)
            within_threshold = sum(path_errors < thresholds(t)) / length(path_errors) * 100;
            path_following_stats.(mode).(['within_' num2str(thresholds(t)*100) 'cm']) = within_threshold;
        end
        
        % Display statistics
        disp(['=== ' mode ' Ground Truth Path Analysis ===']);
        disp(['Mean deviation from planned path: ' num2str(path_following_stats.(mode).mean_error) ' m']);
        disp(['Maximum deviation from planned path: ' num2str(path_following_stats.(mode).max_error) ' m']);
        disp(['Total path length: ' num2str(path_following_stats.(mode).path_length) ' m']);
        disp(['Path smoothness (avg heading change): ' num2str(path_following_stats.(mode).smoothness) ' rad']);
        for t = 1:length(thresholds)
            field = ['within_' num2str(thresholds(t)*100) 'cm'];
            disp(['Time within ' num2str(thresholds(t)) 'm: ' ...
                  num2str(path_following_stats.(mode).(field)) '%']);
        end
        disp(' ');
    end
    
    % Finish main plot
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Ground Truth Path Comparison');
    legend('Location', 'best');
    axis equal;
    grid on;
    
    % Create comparative bar plots in bottom half
    modes = varargin;
    mean_errors = zeros(1, length(modes));
    max_errors = zeros(1, length(modes));
    
    for i = 1:length(modes)
        mean_errors(i) = path_following_stats.(modes{i}).mean_error;
        max_errors(i) = path_following_stats.(modes{i}).max_error;
    end
    
    % Mean error plot
    subplot(2,2,3);
    bar(mean_errors);
    set(gca, 'XTickLabel', modes);
    title('Mean Path Deviation');
    ylabel('Error (m)');
    grid on;
    
    % Max error plot
    subplot(2,2,4);
    bar(max_errors);
    set(gca, 'XTickLabel', modes);
    title('Maximum Path Deviation');
    ylabel('Error (m)');
    grid on;
end