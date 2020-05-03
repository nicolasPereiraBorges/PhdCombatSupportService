function curveSegments = GetCurveSegments(points, horizontalChange)
% function curveSegments = GetCurveSegments(points, horizontalChange)
% Ge curve segments given change points obtained by 
% function horizontalChange = FindHorizontalChangePoints(points)
% Input:  points - matrix (n x 2) of coordinates x and y
%         horizontalChange - matrix n x 1 of boolean
% Output: matrix m x 5 of curveSegments [startPointId, endPointId, cx, cy, radius]
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 13/02/2017

%     figure; plot(points(:,1), points(:,2), 'b', 'linewidth', 2);
%     hold all;
    % Get parameters
    param = Get_Parameters();
    pathSmoothingParam = param.PathSmoothing;    
    % Get number of points   
    nPoints = size(points, 1);    
    startId = 1;
    % Allocate memory for segments    
    curveSegments = zeros(nPoints, 5); %[startPointId, endPointId, cx, cy, radius]
    idSegments = 1;
    % For each point
    for i = 2 : nPoints        
        % Check if is a horizontal change point        
        % And if there's at least 3 points between start id and current id
        if horizontalChange(i) == 1 && (i - startId) > 3            
            endId = nPoints;
            % Check points after the horizontal change
            j = i;
            while j < (nPoints)
                % If found a new horizontal change 
                % And if there's at least 3 points between old and new
                % horizontal change point
                if horizontalChange(j) == 1 && (j - i) > 3
                    endId = j-1;
                    break;
                end
                j = j + 1;
            end            
            
            %Get number of points before and after horizontal change
            nBefore = i - startId;
            nAfter = endId - i;
            % Get min
            minPoints = min(nBefore, nAfter);            
            if minPoints > pathSmoothingParam.SegmentationMinPoints
                % Get 25% of points %
                nPointsRef = round(pathSmoothingParam.SegmentationPointsPercentual*minPoints);
            else
                nPointsRef = minPoints;
            end
            % Get curve points
            startPointsIds = i-nPointsRef:i-1;
            endPointsIds = i:i+nPointsRef;                        
            x = [points(startPointsIds,1); points(endPointsIds,1)];
            y = [points(startPointsIds,2); points(endPointsIds,2)];
            %plot(x,y, 'r', 'linewidth', 2);
            % Apply  circle fitting
            [xc,yc,radius] = Circfit(x,y);
            % Generate segment
            curveSegments(idSegments,1:5) = [startPointsIds(1), endPointsIds(end), xc, yc, radius];            
            idSegments = idSegments + 1;
            startId = i+1+nPointsRef + 1;            
        end
    end
    
    curveSegments = curveSegments(1:idSegments-1, :);
end

