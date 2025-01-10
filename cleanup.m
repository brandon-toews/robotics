disp('Cleaning up...');
% Stop all timers
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
%     end
% catch e
%     disp(e.message);
% end

% Delete the robot object
delete(robot);

delete(node);

disp('Script terminated cleanly.');