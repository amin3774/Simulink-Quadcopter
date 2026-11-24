
function plot3dir(x, y, z, numArrows)
% PLOT3DIR Plot a 3D trajectory with direction arrows.
%
% Usage:
%   plot3dir(x, y, z)
%   plot3dir(x, y, z, numArrows)
%
% Example:
%   plot3dir(out.X(:,1), out.X(:,2), out.X(:,3))

    if nargin < 4
        numArrows = 7;
    end

    % Extract numeric data if inputs are timeseries
    if isa(x, 'timeseries')
        x = x.Data;
    end
    
    if isa(y, 'timeseries')
        y = y.Data;
    end
    
    if isa(z, 'timeseries')
        z = z.Data;
    end
    
    % Convert to column vectors
    x = x(:);
    y = y(:);
    z = z(:);

    % Remove non-finite samples
    valid = isfinite(x) & isfinite(y) & isfinite(z);
    x = x(valid);
    y = y(valid);
    z = z(valid);

    % Calculate cumulative distance
    d = sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2);
    s = [0; cumsum(d)];

    % Remove duplicate distances
    [s, ia] = unique(s, 'stable');
    x = x(ia);
    y = y(ia);
    z = z(ia);

    if numel(s) < 2 || s(end) <= 0
        error('Trajectory must contain at least two distinct points.');
    end

    % Plot trajectory
    figure
    hTrajectory = plot3(x, y, z, 'b-', 'LineWidth', 2);
    hold on
    grid on
    axis equal

    % Arrow settings
    arrowLength = 0.08 * s(end);
    arrowPos = linspace(0.12*s(end), 0.88*s(end), numArrows);
    delta = 0.01 * s(end);

    hArrow = gobjects(1);

    for k = 1:numArrows

        p = arrowPos(k);

        p1 = max(0, p-delta);
        p2 = min(s(end), p+delta);

        P1 = [interp1(s,x,p1), interp1(s,y,p1), interp1(s,z,p1)];
        P2 = [interp1(s,x,p2), interp1(s,y,p2), interp1(s,z,p2)];

        % Tangent direction
        v = P2-P1;
        vNorm = norm(v);

        if vNorm < 1e-12
            continue
        end

        v = v/vNorm;

        % Position on trajectory
        P = [interp1(s,x,p), interp1(s,y,p), interp1(s,z,p)];

        % Draw direction arrow
        hArrow = quiver3( ...
            P(1)-0.5*arrowLength*v(1), ...
            P(2)-0.5*arrowLength*v(2), ...
            P(3)-0.5*arrowLength*v(3), ...
            arrowLength*v(1), ...
            arrowLength*v(2), ...
            arrowLength*v(3), ...
            0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    end

    % Start and end markers
    hStart = plot3(x(1), y(1), z(1), 'go', ...
        'MarkerFaceColor', 'g', 'MarkerSize', 9);

    hEnd = plot3(x(end), y(end), z(end), 'ro', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 9);

    xlabel('X (m)')
    ylabel('Y (m)')
    zlabel('Z (m)')
    title('3D Trajectory with Movement Direction')

    if isgraphics(hArrow)
        legend([hTrajectory, hArrow, hStart, hEnd], ...
            {'Trajectory', 'Direction', 'Start', 'End'});
    else
        legend([hTrajectory, hStart, hEnd], ...
            {'Trajectory', 'Start', 'End'});
    end

    view(3)
    hold off
end
