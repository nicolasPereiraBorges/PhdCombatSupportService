function path = GetSmoothedPath(points, curveSegments)
% function path = GetSmoothedPath(points, curveSegments)
% Get smoothed path given points and curve segments obatined on
% function curveSegments = GetCurveSegments(points, horizontalChange)
% Input:  points - matrix (n x 2) of coordinates x and y
%         curveSegments - matrix m x 5 [startPointId, endPointId, cx, cy, radius]
% Output: path - matrix (n x 2) of coordinates x and y 
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 14/02/2017

    path = repmat(points,1);
    % Get number of segments
    nSegments = size(curveSegments, 1);
    % For each segment
    for i = 1: nSegments
       % Get curveSegment
       segment = curveSegments(i, :);
       % Get curve points
       curvePoints = GetCurvePoints(points, segment);
       path(segment(1):segment(2),1:2) = curvePoints;       
    end
       
end