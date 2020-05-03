function threatsDetected = CheckThreatsWithRWR(uav, threats)
    % function threatsDetected = CheckThreatsWithRWR(uav , threats)
% Detect threats with rwr
% Parameters uav: uav struct
%            threats: (matrix nx1 of struct threat)
% Output:   threatsDetected: (matrix nx1 of struct threat)
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 24/01/2017

    % Get number of threats
    numberOfThreats = size(threats, 1);
    threatsDetected = [];
    % For each intruder
    for k = 1: numberOfThreats
        % Calculate distance from intruder to uav        
        dist = CalculateDistance(uav.x, uav.y, threats(k).cx, threats(k).cy);
       
        % If distance is lower than uav's rwr range    
        if dist < (uav.rwrRange+threats(k).range)
                        
                % Add threat        
                [xInt,yInt] = GeneratePointsCircle(dist, threats(k).cx, threats(k).cy);        
                intruder = struct('range', threats(k).range, ...
                                  'cx', threats(k).cx, ...
                                  'cy', threats(k).cy, ...
                                  'x', xInt, ...
                                  'y', yInt, ...
                                  'weaponRange', threats(k).weaponRange);
                threatsDetected = [threatsDetected; intruder];            
        end
    end

end
