function [uavs, endSimulation] = ExecuteUavsActions(uavs, threats, currentTime,constantParameters)
% function [uavs, endSimulation] = ExecuteUavsActions(uavs, threats, currentTime)
% Execute all uavs actions during one iteraction
% Input: uavs (matrix nx1 of struct uav) - All uavs in the scenario
%        threats (matrix nx1 of struct threat) - All threats in the
%        scenario
%        currentTime - Current simulation time
% Output: uavs = (matrix nx1 of struct uav) -> updated uavs
%         endSimulation (1 if simulation finished and 0 not finished yet)        
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 26/01/2017   
            
    % Get all parameters
    param = Get_Parameters();
    % Get only uav parameters
    uavParam = param.Uav;
    % Get sufficient distance to target
    sufficientDistance = uavParam.sufficientDistanceToGoal;
    % Get number of uavs
    numberOfUavs = size(uavs,1);    %fprintf('Time - %d\n', currentTime);
    % Create graph (network)
    graph = CreateGraph(uavs);   
    % Plot iteraction
    PlotScenario(uavs, graph, threats, currentTime, 0);  
    % uavs = RegroupFormationIds(uavs, graph);
    % For each uav 

    %uavs = ChangeUAVsSpeed(uavs, graph);
    for index = 1: numberOfUavs         
        % Create graph (network)
        graph = CreateGraph(uavs);   
        % Send current position to its neighbours
        uavs = UpdateLocalLeaderPosition(uavs, uavs(index));              
        % If uav didn't reach goal or is destroyed        
        if uavs(index).distance > sufficientDistance & uavs(index).destroyed == 0           
            % Check threats in using rwr                   
            uavs = CheckThreatsAnUpdateDetectedThreatsList(uavs, index, threats, graph, constantParameters);                        
            % Regroup formation         
            uavs = RegroupFormation(uavs(index), uavs, graph);                             
            % Calculate path      
            try
                uavs = CalculatePath(uavs, index, 0);                              
            catch
                a = 1;
            end
        else
            
             if uavs(index).destroyed == 0
                uavs = UAVFail(uavs, index);                
             end          
        end                      
    end       
    % Reduce the distance between the uavs paths (for unstructured flight formation)     
    uavs = MergePathsBasedOnCurveSimilarity(uavs);        
    %PlotScenario(uavs, graph, threats, 1);
    graph = CreateGraph(uavs);  % necessary because an UAV may turn off radar
    % Update UAVs position based on combined control laws          
    if uavs(1).communicationRadarOn == 0
        cUAVS = repmat(uavs,1);
    end
    uavs = UpdateUAVsPosition(uavs, graph, constantParameters);                    
    
    % If all uavs are destroyed or reach the goal end simulation
    endSimulation = sum([uavs.destroyed]) == numberOfUavs;          
end

function newUAVs = UAVFail(uavs, index)

    newUAVs = repmat(uavs,1);
    newUAVs(index).destroyed = 1;
    
    
    if ~isempty(newUAVs(index).formationPredecessor)
        predecessor = GetIndexOfUav(newUAVs, uavs(index).formationPredecessor);
        newUAVs(predecessor).formationSuccessor = [];        
    end
    
    if ~isempty(newUAVs(index).formationSuccessor)
        newLeader = GetIndexOfUav(newUAVs, newUAVs(index).formationSuccessor);
        newUAVs(newLeader).localLeaderPosition = newUAVs(newLeader).goal;
        newUAVs(newLeader).formationPredecessor = []; 
        newUAVs(newLeader).formationId = 1;
        newUAVs(newLeader).path = [];
      %  newUAVs(newLeader).speed = 100;
        newUAVs = CalculatePath(newUAVs, newLeader, 0);
        graph = CreateGraph(newUAVs);
        newUAVs = RegroupFormation(newUAVs(newLeader), newUAVs, graph);                                                                 
    end
end