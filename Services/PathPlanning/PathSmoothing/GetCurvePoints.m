function curvePoints = GetCurvePoints(points, segment)
% function points = GetCurvePoints(segment)
% Get curve points given curveSegment 
% input: points - matrix (n x 2) of coordinates x and y 
%        segment - CurveSegment [startPointId, endPointId, cx, cy, radius]
% Output: curvePoints: Generated points (matrix n2x of points [x,y])
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 14/02/2017
        
    startPoint = points(segment(1), :);        
    endPoint = points(segment(2), :);    
    pRef = points(round((segment(2)+ segment(1))/2), :);
    nPoints = segment(2) - segment(1) + 1;
    cx = segment(3);
    cy = segment(4);
    radius = segment(5);            
    
    %[x,y] = GeneratePointsCircle(radius, cx, cy);
    %plot(x,y, 'r', 'linewidth', 3);
   
    % Get curve Direction    
    direction = ispolycw([startPoint(1), pRef(1), endPoint(1)], [startPoint(2), pRef(2), endPoint(2)]);
    
    % Get theta
    theta1 = CalculateAngleBetweenTwoPoints([cx,cy], startPoint);
    theta2 = CalculateAngleBetweenTwoPoints([cx,cy], endPoint);   
    
    if direction == 1 % clock-wise
        if theta1 < 0 && theta2 > 0
            theta1 = theta1 + 2*pi;
        end           
    else % anti-clockwise
        if theta2 > 0 && theta1 > 0
           theta2 = theta2 - 2*pi;
        end
        if theta1 > 0
            theta1 = theta1 - 2*pi;
        end        
    end
            
    diff = abs(theta1-theta2);
    ratio = diff/nPoints;
    
    if theta1 < theta2
        th = theta1:ratio:theta2;
    else
        th = theta1:-ratio:theta2;
    end
            
    x = radius * cos(th) + cx;    
    y = radius * sin(th) + cy;      
    curvePoints = [x(1:end-1);y(1:end-1)]';          
    %plot(curvePoints(:,1), curvePoints(:,2), 'g', 'linewidth', 3);
end
