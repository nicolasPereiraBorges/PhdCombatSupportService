function newPoints = ApplyFixedSpace(points, distance)
% function newPoints = ApplyFixedSpace(points, distance)
% Generate fixed space points
% Parameters: points: matrix N x 2 (lon, lat)
%             distance: meters (float)
% Output: newPoints: matrix N x 2 (lon, lat)
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 04/09/2016

%newPoints = points(2:end,:);
%distance = 10;
% check number of points
nPoints = size(points, 1);

% if there is a single point
if nPoints < 2 %|| CalculateDistance(points(1,1), points(1,2), points(2,1), points(2,2)) < distance
    % return point
    newPoints = points;
    return;
end

% First point is equal 
%newPoints(1, 1:2) = points(1, :);
currentDistance = 0.0;

point1 = Position3D(points(1,1), points(1,2));
point2 = Position3D(points(2,1), points(2,2));
% Calculate distance between last reached point to p2
dist = CalculateDistance(point1, point2);
[a,b] = GetLineEquation(points(1,1), points(1,2), points(2,1), points(2,2));
ratio = dist/distance;
ratio = (points(2,1)-points(1,1))/ratio;
x = points(1,1)+ratio;
y = polyval([a,b], x);
newPoints(1, 1:2) = [x,y];
% From first point to n-1 point
for i = 1: nPoints - 1
    % Get points    
    p1 = points(i, :);
    p2 = points(i+1, :);
    point1 = Position3D(p1(1), p1(2));
    point2 = Position3D(p2(1), p2(2));
    % Calculate distance between points
    pointsDistance = currentDistance + CalculateDistance(point1, point2);   
    currentDistance = pointsDistance;
    if pointsDistance > distance
        % Calculate line equation
        if p1(1) ~= p2(1) % if have different x
            [a,b] = GetLineEquation(p1(1), p1(2), p2(1), p2(2));
            % Calculate x ratio between p1 and p2
            ratio = pointsDistance/distance;
            ratio = (p2(1)-p1(1))/ratio;
            %ratio = CalculateRatio(p1, p2, pointsDistance, distance - currentDistance);
            % Interpolate
            x = (p1(1)+ratio:ratio:p2(1))';
            y = polyval([a,b], x);
            newPoints = [newPoints; [x,y]];
            
            point1 = Position3D(x(end), y(end));
            point2 = Position3D(p2(1), p2(2));
            % Calculate distance between last reached point to p2
            currentDistance = CalculateDistance(point1, point2);
        else
            [a,b] = GetLineEquation(p1(2), p1(1), p2(2), p2(1));
            % Calculate x ratio between p1 and p2
        ratio = pointsDistance/distance;
        ratio = (p2(2)-p1(2))/ratio;
        %ratio = CalculateRatio(p1, p2, pointsDistance, distance - currentDistance);
        % Interpolate
        x = (p1(2)+ratio:ratio:p2(2))';
        y = polyval([a,b], x);
        newPoints = [newPoints; [y,x]];
        
        point1 = Position3D(x(end), y(end));
        point2 = Position3D(p2(1), p2(2));
        % Calculate distance between last reached point to p2
        currentDistance = CalculateDistance(point1, point2);
        end
        
    end    
end

newPoints = [newPoints;points(end,:)];
if newPoints(1,1) == newPoints(2,1) && newPoints(1,2) == newPoints(2,2)
    newPoints = newPoints(2:end, :);
end
if isnan(newPoints(1,1))
    newPoints = newPoints(2:end, :);
end
end



