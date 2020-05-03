function goal = CalculateGoalToLeaveThreat(uavs, index)
% function uavs = CalculateGoalToLeaveThreat(uavs, index, uav)
% Calculate path to leave threat of the UAV that detects a threat
% Input: uavs: matrix (np1(1)) of struct UAV
%        index: index of uav in uavs 
% Output goal: [x,y]  of the goal position of the uav to leave the threat
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 11/05/2017        

        % Get parameters
        param = Get_Parameters();
        flightFormationParam = param.FlightFormation();
        % If uavs are flying in trail formation
        if flightFormationParam.lineFormation == 1
            try
             goal = CalculateGoalToLeaveThreatTrailFormation(uavs, index);
            catch
               goal = uavs(index).goal;
            end
                
        else % Unstructured formation
           goal = CalculateGoalToLeaveThreatUnstructuredFormation(uavs, index);      
        end
end
