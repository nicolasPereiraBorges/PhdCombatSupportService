function [a,b] = GetLineEquation(x1, y1, x2, y2)
% function [a,b] = GetLineEquation(x1, y1, x2, y2)
% Calculate line equation that pass trought [x1,y1] and [x2,y2] on the form
% fx = ax + b
% Input x1: x position of point1
%       y1: y position of point1
%       x2: x position of point2
%       y2: y position of point2
% Output a = 
%Technological Institute of Aeronautics
%Author: Nicolas Pereira Borges - nicolas@ita.br
%Date: 25/01/201

% Calculate angular coefficient
if y1==y2
    y1 = y1+1E-30;
end
a = (y1-y2)/(x1-x2);
%y - y0 = + m(x - x0)
b = y1 - a*x1;

end
