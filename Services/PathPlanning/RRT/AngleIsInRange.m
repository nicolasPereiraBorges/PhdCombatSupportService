function result = AngleIsInRange(point1, point2, angleRef)
% function result = AngleIsInRange(point1, point2, angleRef)
% Check if angle between point1 and point2 is in range, compared with
% angleRef
% Parameters: point1 (x,y)
%             point2 (x,y)
%             angleRef  radians
% Output: result: boolean 
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 21/02/2017

        % Get all parameters
        %param = Get_Parameters();
        % Get only flight path parameters
        %flightPathParam = param.PathPlanning;
        % Calculate angle with new point
        
        result = 1;
        return;
         
        angle = CalculateAngleBetweenTwoPoints(point1, point2);
        angle = abs(angle);
        diff = abs(abs(angle) - angleRef);
      
        variation = 2*pi;
        %flightPathParam.angleVariation;
        if angle < 0 && angleRef < 0
            angle = abs(angle);
            angleRef = abs(angleRef);
        elseif  angle < 0 && angleRef > 0 || ...
                angle > 0 && angleRef < 0
            variation = variation / 2;
        end
        
        diff = max(angle,angleRef) - min(angle, angleRef);
        % If new angle is not in range of current angle
        if diff > variation%angle < (angleRef - variation) || angle  > (angleRef + variation)
            % Needs to create new point
            result = 0;
        else 
            result = 1;
        end
        
end