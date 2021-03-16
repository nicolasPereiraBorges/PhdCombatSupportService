function [p1, p2] = GetPerpendicularPoints(uav, threat)
% function [p1, p2] = GetPerpendicularPoints(uav, threat)
% Get points tangent to threat virtual range of the line that is
% perpendicular to line that pass trough uav current position and its last
% position
% input:  uav - struct uav
%         threat - threat that uav is trying to avoid
% output: p1 - [x,y] of first point
%         p2 - [x,y] of second point
%         midPoint = [xMid, yMid]
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 11/05/2017 
   
    lastMovements = uav.GetPathHistoryAsArray();
    factor = 1.3;
    % Get midPoint of line to end of threat    
    xX = lastMovements(:,1);
    yY = lastMovements(:,2);
    eq = polyfit(xX,yY, 1);
    % Perp Equation
    perpA = -1/eq(1);
    % Passes trough center of threat
    perpB = threat.Position.Y - (threat.Position.X  * perpA);
    perpEq = [perpA, perpB];
    xLeft = threat.Position.X - 10;
    yLeft = polyval(perpEq, xLeft);
    xRight = threat.Position.X + 10;
    yRight = polyval(perpEq, xRight);
    
    [x1,y1] = ProjectPoint(threat.Position.X, threat.Position.Y, xLeft, yLeft, threat.DetectionRange*factor);
    [x2,y2] = ProjectPoint(threat.Position.X, threat.Position.Y, xRight, yRight, threat.DetectionRange*factor);
       
    p1 = [x1,y1];
    p2 = [x2,y2];
end


function [xres, yres] = ProjectPoint(x1, y1, x2, y2, distance)
% function [xres, yres] = ProjectPoint(x1, y1, x2, y2, distance)
% Project point from [x1,y1] to [x2,y2] in a straigth with a
% distance threshold
% Input x1: x position of point1
%       y1: y position of point1
%       x2: x position of point2
%       y2: y position of point2
%       distance: distance from vertex to goal 
%Output:   xres: x position of projected point
%          yres: y position of projected point
%Technological Institute of Aeronautics
%Author: Nicolas Pereira Borges - nicolas@ita.br
%Date: 25/01/2017

% if p1 and p2 have same x and different y
if x1 == x2 && y1 ~= y2
    % Invert x and y
    aux = x1;
    x1 = y1;
    y1 = aux;

    aux = x2;
    x2 = y1;
    y2 = aux;

    reverse = 1;
else    
    reverse  = 0;
end
% Get line equation
[a,b] = GetLineEquation(x1, y1, x2, y2);
% Calculate fx at the projected distance
xres_aux = x1+((x2-x1)* distance) / CalculateDistance(Position3D(x1, y1), Position3D(x2, y2));
yres_aux = CalculateFx(a, b, xres_aux);
   
if reverse == 1
    xres = yres_aux;
    yres = xres_aux;
else
    xres = xres_aux;
    yres = yres_aux;
end

end