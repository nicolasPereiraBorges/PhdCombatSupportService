function threats = CreateThreats(gcs, nThreats, groundForces)
% function threats = CreateThreats(groundForces)
% Create threats in the scenario given parameters
% obtained from funciton Parameters()
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 02/05/2020

% Get parameters
parameters = Parameters();
% Threatss parameters
%nThreats = parameters.ScenarioNumberOfThreats;
detectionRange = parameters.ThreatsDetectionRange;
weaponRange = parameters.ThreatsWeaponRange;
% Create List
threats = List();
% Create UAVs
for i = 1: nThreats
    threat = Threat();
    threat.Id = i;
    threat.DetectionRange = detectionRange;
    threat.WeaponRange = weaponRange;
    threat.Position =  GenerateValidPosition(parameters, threats, gcs, groundForces);
    threats = threats.AddLast(threat);
end
end

function pos = GenerateValidPosition(parameters, threats, gcs, groundForces)

minDistanceToGCS = parameters.ScenarioMinDistanceThreatsAndGCS;
minDistanceToThreats = parameters.ScenarioMinDistanceBetweenThreats;
minDistanceToGFS = parameters.ScenarioMinDistanceThreatsAndGFs;
xlim = [0, parameters.ScenarioWidth];
ylim = [0, parameters.ScenarioHeight];
zlim = [0, parameters.ScenarioDeepth];

valid = 0;
while(valid == 0)
    valid = 1;
    pos = GenerateRandomPosition(xlim, ylim, zlim);
    
    % Distance to GCS
    if (CalculateDistance(pos, gcs.Position) < minDistanceToGCS)
        valid = 0;
        continue;
    end
    % Distance to GFs
    for i = 1:groundForces.Count()
        if (CalculateDistance(pos, groundForces.Value(i).Position) < minDistanceToGFS)
            valid = 0;            
        end
    end            
    % Distance to other threats
    for i = 1:threats.Count()
        d = CalculateDistance(pos, threats.Value(i).Position);
        if (d < minDistanceToThreats)
            valid = 0;            
        end
    end
      
end


end

