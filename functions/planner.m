function [n, stop] = planner(waypoints, position, capture_threshold, detections, reset)
% WAYPOINT_SELECTOR
% Maintains a persistent waypoint index for an RVWP-style controller.
% Inputs
%   waypoints         : Nx2 [x y]
%   position          : [x y]
%   capture_threshold : scalar (distance to switch to the next waypoint)
%   reset             : optional logical, when true resets internal state
%   detections        : obstacle detection data (x,y,id) 
% Outputs
%   n     : current segment start index (uses waypoints n -> n+1)
%   stop  : 1 if the final waypoint is considered reached

    % persistent index
    persistent idx
    obstacle_radius = 2;
    detection_threshold = 2;
    if nargin >= 4 && reset
        idx = [];
    end
    if isempty(idx)
        idx = 1;
    end

    N = size(waypoints, 1);
    stop = 0;

    % already at or past the final waypoint
    if idx >= N
        stop = 1;
        n = idx;
        return
    end

    % distance to next waypoint
    x = position(1);  y = position(2);
    d_next = hypot(x - waypoints(idx+1,1), y - waypoints(idx+1,2));

    % advance when close enough
    if d_next < capture_threshold
        idx = min(idx + 1, N);
        if idx >= N
            stop = 1;
        end
    end

    % % ===== Detection print =====
    % if ~isempty(detections)
    %     % Find obstacles within detection_threshold distance
    %     for i = 1:size(detections,1)
    %         dx = x - detections(i,1);
    %         dy = y - detections(i,2);
    %         dist = hypot(dx, dy);
    % 
    %         fprintf('[Planner] Detection made at (%.2f, %.2f)\n', ...
    %                 detections(i,1), detections(i,2));
    %     end
    % end


    n = idx;
end
