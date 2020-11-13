function path = ApplyPathSmoothing(points)
% function path = ApplyPathSmoothing(points)
% Apply path smoothing using horizontal segmentation procedure
% input: points - matrix (n x 2) of coordinates x and y 
% Output: path - matrix (n x 2) of coordinates x and y 
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 14/02/2017
        
    try
    %path = points;
    %return;
    if size(points,1) < 12
        path = points;
        return;
    end
    % Apply horizontal segmentation
    h = FindHorizontalChangePoints(points);    
    % Get curve segments
    segments = GetCurveSegments(points, h);
    % Reconstitute curves
    path = GetSmoothedPath(points, segments);
    % Apply Moving average
    for i = 1:5
        path = movmean(path,5);
    end            
    catch
        path = points;
    end
end