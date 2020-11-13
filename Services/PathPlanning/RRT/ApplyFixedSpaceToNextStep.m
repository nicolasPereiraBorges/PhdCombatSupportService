function newPoints = ApplyFixedSpaceToNextStep(points, speedFraction, refPoint)
% function newPoints = ApplyFixedSpace(points, distance)
% Generate fixed space points
% Parameters: points: matrix N x 2 (lon, lat)
%             distance: meters (float)
% Output: newPoints: matrix N x 2 (lon, lat)
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 04/09/2016

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
fullDistance = 0.0;
dist = CalculateDistance(points(1,1), points(1,2), points(2,1), points(2,2));
[a,b] = GetLineEquation(points(1,1), points(1,2), points(2,1), points(2,2));
ratio = dist/speedFraction;
ratio = (points(2,1)-points(1,1))/ratio;
x = points(1,1)+ratio;
y = polyval([a,b], x);
newPoints(1, 1:2) = [x,y];
% From first point to n-1 point
for i = 1: nPoints - 1
    % Get points
    p1 = points(i, :);
    p2 = points(i+1, :);
    % Calculate distance between points
    pointsDistance = currentDistance + CalculateDistance(p1(1), p1(2), p2(1), p2(2));
    currentDistance = pointsDistance;
    
    if pointsDistance > speedFraction
        % Calculate line equation
        if p1(1) ~= p2(1) % if have different x
            [a,b] = GetLineEquation(p1(1), p1(2), p2(1), p2(2));
            % Calculate x ratio between p1 and p2
            ratio = pointsDistance/speedFraction;
            ratio = (p2(1)-p1(1))/ratio;
            %ratio = CalculateRatio(p1, p2, pointsDistance, distance - currentDistance);
            % Interpolate
            x = (p1(1)+ratio:ratio:p2(1))';
            y = polyval([a,b], x);
            newPoints = [newPoints; [x,y]];
            % Calculate distance between last reached point to p2
            currentDistance = CalculateDistance(x(end), y(end), p2(1), p2(2));
            dist = CalculateDistance(refPoint(1), refPoint(2), x,y);
            [l,~] = find(dist < 10);
            if ~isempty(l)
                newPoints = newPoints(1:l,:);
                return;
            end          
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
            fullDistance = fullDistance + currentDistance;
            % Calculate distance between last reached point to p2
            currentDistance = CalculateDistance(x(end), y(end), p2(2), p2(1));
            %fprintf('dist - %.2f\n', CalculateDistance(refPoint(1), refPoint(2), newPoints(end,1), newPoints(end,2)));
            dist = CalculateDistance(refPoint(1), refPoint(2), y,x);
            [l,c] = find(dist < 10);
            if ~isempty(l)
                newPoints = newPoints(1:l,:);
                return;
            end
        end
        
    end
end

newPoints = [newPoints;points(end,:)];
%newPoints = newPoints(2:end, :);

end





