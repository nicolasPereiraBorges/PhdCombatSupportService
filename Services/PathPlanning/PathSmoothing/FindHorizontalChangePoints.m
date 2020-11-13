function horizontalChange = FindHorizontalChangePoints(points)
% function newPoints = FindHorizontalChangePoints(points)
% Get horizontal change points by variation of angular coefficient
% Input:  points - matrix (n x 2) of coordinates x and y
% Output: horizontalChange - matrix n x 1 of boolean
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 13/02/2017

    nPoints = size(points, 1);            
    startId = 1;
    horizontalChange = zeros(nPoints,1);
    previous = (points(2,2)-points(1,2))/(points(2,1)-points(1,1));   
    i = 2;
    while isnan(previous)
        previous = (points(i,2)-points(1,2))/(points(i,1)-points(1,1));   
        i = i + 1;
    end
    % for each points
    for i = 2:nPoints - 1   
       current = points(i,:); 
       endId = i+1;
       next = points(endId,:);
       % Calculate function   
       current = (current(2)-next(2))/(current(1)-next(1));   
       % If change line   
       if abs(previous - current) > 1E-1 && startId ~= endId       
           %plot(points(i,1), points(i,2), '.b', 'markersize', 20);
           horizontalChange(i) = 1;
           startId = endId;           
           previous = current;
       end
    end

end

