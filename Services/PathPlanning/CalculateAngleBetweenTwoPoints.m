function angle = CalculateAngleBetweenTwoPoints(p1, p2)
% function angle = CalculateAngleBetweenTwoPoints(p1, p2)
% Calculate angle between two points
% Inputs: p1 (x,y)
%         p2 (x,y)
% Output angle - double (radians)
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 20/02/2017
    
    angle = atan2(p2(2) - p1(2), p2(1) - p1(1));
    
end

