function groundForces = CreateGroundForces(gcs)
% function threats = CreateThreats(groundForces)
% Create threats in the scenario given parameters
% obtained from funciton Parameters()
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 02/05/2020

% Get parameters
parameters = Parameters();
% Threatss parameters
nGroundForces = parameters.ScenarioNumberOfGroundForces;
% Create List
groundForces = List();
% Create UAVs
for i = 1: nGroundForces
    gf = GroundForce();
    gf.Id = i;    
    gf.Position =  GenerateValidPosition(parameters, gcs, groundForces);
    groundForces = groundForces.AddLast(gf);
end
end

function pos = GenerateValidPosition(parameters, gcs, groundForces)

minDistanceToGCS = parameters.ScenarioMinDistanceGCSAndGFs;
minDistanceToGFs = parameters.ScenarioMinDistanceGFs;
xlim = [0, parameters.ScenarioWidth];
ylim = [0, parameters.ScenarioHeight];
zlim = [0, parameters.ScenarioDeepth];


while(1)
    valid = 1;
    pos = GenerateRandomPosition(xlim, ylim, zlim);
    
    % Distance to GCS
    if (CalculateDistance(pos, gcs.Position) < minDistanceToGCS)
        continue;
    end
    
    % Distance to other gfs
    for i = 1:groundForces.Count()
        if (CalculateDistance(pos, groundForces.Value(i).Position) < minDistanceToGFs)
            valid = 0;
            break;
        end
    end
    
    if (valid == 1)
        break;       
    end
end


end

