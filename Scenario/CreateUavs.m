function uavs = CreateUavs(gcs)
% function uavs = CreateUavs(param)
% Create uavs in the scenario given parameters
% obtained from funciton Parameters()
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 02/05/2020
        
    % Get parameters
    parameters = Parameters();        
    % UAVs parameters
    nUAVs = parameters.ScenarioNumberOfUAVs;
    speed = parameters.UAVsSpeed;
    % Create List
    uavs = List();    
    % Create UAVs
    for i = 1: nUAVs
        uav = UAV();
        uav.Id = i;
        %uav.Angle = parameters.angleVariation;
        uav.Speed = speed;
        uav.Gcs = gcs;        
        uav.Position = gcs.Position;        
        uavs = uavs.AddLast(uav);
    end
end