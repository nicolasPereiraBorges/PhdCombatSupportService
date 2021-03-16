function points = ReducePath(points, threats)
% function points = ReducePath(points, threats)
% Get min path given original rrt path
% Input: points (matrix nx2) [x,y]
%        threats: matrix nx2 of struct threats
% Output: modified points
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 26/01/2017
% Get number of threats
param = Parameters();
angleRef = param.angleVariation;

% Set tolerance for distance from point to threat
tolerance = 0.2;
%Reduce Path
for i = 1:size(points(:, 1), 1)
    j = size(points(:, 1), 1);
    while j > (i+1) 
        %p1ToClose = CheckIfPointIsToCloseToThreat(points(i, :), threats, tolerance);
        p2ToClose = CheckIfPointIsToCloseToThreat(points(j,:), threats, tolerance);        
        intercepts = LineIntersectsObstacle(points(i,1:2), points(j,1:2), threats);             
        if ~intercepts %&& ~p2ToClose
            %angle = CalculateAngleBetweenTwoPoints(points(i, :), points(j,:));
%             if AngleIsInRange(points(i, :), points(j,:), angleRef) == 1            
%                 canReduce = 0;    
%                 if j == size(points(:, 1), 1)                
%                         canReduce = 1;                
%                 else
%                     angle = CalculateAngleBetweenTwoPoints(points(j,:),points(j+1,:));
%                     if AngleIsInRange(points(j,:),points(j+1,:), angleRef) == 1            
%              %           plot([points(j,1), points(j+1,1)], [points(j,2), points(j+1,2)], 'g');
%                         canReduce = 1;
%                     else
%                       %  plot([points(j,1), points(j+1,1)], [points(j,3), points(j+1,2)], 'r');
%                     end
%                 end
                                             
           %if canReduce == 1
                points = [points(1:i, :); points(j:end, :)];            
                break;
          %  end
           % else
                %plot([points(j,1), points(j+1,1)], [points(j,2), points(j+1,2)], 'b');
          %  end
               
   
        else
            %plot([points(i,1),points(j,1)], [points(i,2),points(j,2)], 'r');
        end
        j = j - 1;
    end
end

end

% function pointToClose = CheckIfPointIsToCloseToThreat(point, threats, tolerance)
function pointToClose = CheckIfPointIsToCloseToThreat(point, threats, tolerance)
    pointToClose = 0 ;
    return;
    % Get number of threats
    nThreats = size(threats,1);
    % For each threats
    for i = 1: nThreats
        dist = CalculateDistance(point(1), point(2), threats(i).cx, threats(i).cy);
        if dist < threats(i).virtualRange
           pointToClose = 0;
           return;
        end
        if 1 - ((min(threats(i).virtualRange, dist)/max(threats(i).virtualRange, dist))) < tolerance
            pointToClose = 1;
            return
        end
    end
    pointToClose = 0;
             
end

