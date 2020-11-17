function intersects = LineIntersectsObstacle(startPoint, goal, obstacles)
% function newPoints = LineIntersectsObstacle(startPoint, goal, obstacles)
% Check if line intersects with some obstacle
% Parameters: startPoint (lon, lat)
%             goal (lon, lat)    
%             obstacles Nx3 matrix (cx, cy, radius)
% Output: intercects: boolean 
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 31/08/2016
    
    intersects = false;
    % Get coordinates from points
    x1 = startPoint(1);
    y1 = startPoint(2);
    x2 = goal(1);
    y2 = goal(2);       
    
    % If x1 == x2 is impossible generate a line equation
    if x1 == x2
        % Exit
        return;
    end
    
    % Calculate line between start point and goal
    [a,b] = GetLineEquation(x1, y1, x2, y2);
    % Calculate fx
    y_intercept = CalculateFx(a,b,0);
    % Get number of obstacles
    nObstacles = size(obstacles,1);
    if nObstacles == 0
        return;
    end
    % For each obstacle
    for i = 1: nObstacles
       % Get obstacle
       c = obstacles(i,:);       
       % Get intersection between line and circle
       [x,y] = linecirc(a, y_intercept, c(1), c(2), c(3));              
       % If there is no interception
       if isnan(x(1))
           continue; % exit with false
       end
       
       % Check first interception
       if PointInsideDomain([x(1), y(1)], [x1,x2], [y1,y2]) == 1
           intersects = true;
           return;
       end
       
       % Check second interception
       if PointInsideDomain([x(2), y(2)], [x1,x2], [y1,y2]) == 1
          intersects = true;
           return;
       end
    end

end


function inside = PointInsideDomain(point, xlim, ylim)
% function inside = PointInsideDomain(point, xlim, ylim)
% Check if point is inside a domain (limit)
% Parameters: point - (x,y)
%             xlim -(x1, x2)
%             ylim -(y1, y2)
% Output: inside: boolean 
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 09/02/2017

    % Get x bounds
    x1 = min(xlim);
    x2 = max(xlim);
    % Get y bounds
    y1 = min(ylim);
    y2 = max(ylim);
    % Check domain
    if point(1) >= x1  && point(1) <= x2  && ... % x domain
            point(2) >= y1  && point(2) <= y2 % y domain
        inside = 1; % point inside domain
    else
        inside = 0; % point outside domain
    end
    
end