function uavs = UpdateUAVsPosition(uavs, graph, constantParameters)
% function uavs = UpdateUAVsPosition(uavs, graph)
% Update uavs position based on a combination of control laws
% Parameters: uavs: matrix(nx1) of struct uav
%             graph: Graph instance
% Output:    uavs: matrix(nx1) of struct uav - Updated positions
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 29/11/2016
    
    % Capture uav networks
    connectedUavs = CaptureUavNetworks(uavs, graph);
    % Get number of connected graphs
    nConnectedUavs = size(connectedUavs,2);
    for i = 1: nConnectedUavs                                               
        % Get new positions based on control laws.
        [factorUX, factorUY] = ApplyControlLaw(connectedUavs{i}, constantParameters);        
        nLocalUavs = size(connectedUavs{i}, 1);
        % For each connected uav
        for j = 1: nLocalUavs            
            % Get uav
            uav = connectedUavs{i}(j);
            if CalculateDistance(uav.x, uav.y, factorUX(j), factorUY(j)) < 1 && ...
                    CalculateDistance(uav.x, uav.y, uav.localLeaderPosition(1), uav.localLeaderPosition(2)) < 5%|| ...
                uav.path = [];
                uav.formationId = 1;
                if uav.communicationRadarOn == 1
                    uav.localLeaderPosition = uav.goal;
                end
                uav.lastPositions = [uav.x, uav.y;uav.lastPositions];
                
                if size(uav.lastPositions,1) > 5
                    uav.lastPositions = uav.lastPositions(1:5, :);
                end
            else
                % Update angle of uav            
                uav.angle = CalculateAngleBetweenTwoPoints([uav.x, uav.y],[factorUX(j), factorUY(j)]);             
                uav.lastPositions = [uav.x, uav.y;uav.lastPositions];
             end

            % Check formation id            
            if uav.formationId ~= 1
                % needs to recalculate path
                uav.path = [];
            end
            % Get index of uav
            indexUav = GetIndexOfUav(uavs, uav.id);            
            % Update position
            uav.x = factorUX(j);
            uav.y = factorUY(j);                        
            % Update distance to goal    
            uav.distance = CalculateDistance(uav.x, uav.y, uav.goal(1), uav.goal(2));
            uavs(indexUav) = uav;
            % Send local leader position
            uavs = UpdateLocalLeaderPosition(uavs, uav);
            % If formationId == 1
            if uav.formationId == 1
                % Send global leader position
                uavs = BroadcastNewGlobalLeaderPosition(uavs, uav, graph);
            end
        end
    end
    
   
end

function [factorUX, factorUY] = GetTargetPosition(connectedUavsLocal)
% function [factorUX, factorUY] = GetTargetPosition(connectedUavs)
% Get target position of all connected uavs connectedUavs{i}
% Input: connectedUavsLocal - connectedUavs{i}
% Output: factorUX: cordinates x of target position
%         factorUY: cordinates y of target position
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 26/01/2017

    % Get number of uavs in the network
    nLocalUavs = size(connectedUavsLocal, 1);
    % Allocate memory for target positions
    factorUX = zeros(1, nLocalUavs);
    factorUY = zeros(1, nLocalUavs);
    % For each uav
    for j = 1: nLocalUavs
        % Get uav
        uav = connectedUavsLocal(j);                 
        % Update position                        
        pos = uav.path(1,:);
        factorUX(j) = pos(1);
        factorUY(j) = pos(2);              
    end
end

function [newX, newY] = ApplyControlLaw(uavs, param)
% function [newX, xewY] = ApplyControlLaw(uavs)
% Calculate next uavs position based on a set of controls law
% Input: uavs (matrix nx1 of struct uav)
% Output: newX: cordinates x of new position
%         newY: cordinates y of new position
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 26/01/2017

    % Get all parameters
    %param = Get_Parameters();
    % Get only simulation parameters
    controlLawsParam = param.ControlLaws;    
    % Get x and y coordinates
    xUavs = [uavs.x];
    yUavs = [uavs.y];
    positions = [xUavs,yUavs]';
    
    % Compute centroids to improve coverage area
    centroidsCoverage = ComputeCentroidsCoverage( [xUavs;yUavs]', controlLawsParam, uavs);    
    % Compute centroids to improve coverage area
    centroidsRobustness = ComputeCentroidsRobustness([xUavs;yUavs]', controlLawsParam, uavs);           
    % Apply control law   
    %p.'Abstol',1e-5,'Reltol',1e-5,'Jacobian','on'        
    [~, positionI]=ode113(@(t,x) ConnectivityCA_controlLaw(x, uavs, centroidsCoverage, centroidsRobustness, param), ...
        [0 1],positions);        
           
    %transform vector into 2-column vector (x axis, y yaxis);
    nUavs = size(uavs,1);
    factorUX2=positionI(:,1:nUavs);
    factorUY2=positionI(:,nUavs+1:end);
    
    % Get last position
    newX = factorUX2(end,:);
    newY = factorUY2(end,:);
    
    
end


function centroids = ComputeCentroidsCoverage(positions, controlLawsParam, uavs)
% function centroids = ComputeCentroidsCoverage(positions, controlLawsParam)
% Compute centroids to coverage area improve using voronoi
% Input: positions - matrix nx2 of uavs position (x,y)
%        controlLawsParam - struct Get_Parameters().ControlLawsParam();
%        uavs - matrix nx1 of struct UAV
% Output: centroidsVoronoi -  matrix nx2 of goal position (x,y)
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 19/07/2017

    try
    if controlLawsParam.adpGainCoverage > 0
        % Get centroids for coverage improvement
        [centroids]=define_centroid_local(positions);
        for i = 1: size(centroids, 1)
            % Check if centroid is inside some threat
            if PointIsInsideSomeThreat(centroids(i,1:2), uavs(i).threats) 
                dist = CalculateDistance(uavs(i).x, uavs(i).y, centroids(i,1), centroids(i,2));
                if dist < uavs(i).speed * 2 && LineIntersectsObstacle([uavs(i).x, uavs(i).y], centroids(i,1:2), uavs(i).threats) == 0
                    % If is inside, use current UAV position 
                    centroids(i, 1:2) = positions(i,:);
                elseif dist >= uavs(i).speed
                        [x,y] = ProjectPoint(uavs(i).x, uavs(i).y, centroids(i,1), centroids(i,2), uavs(i).speed*2);
                    if LineIntersectsObstacle([uavs(i).x, uavs(i).y], [x,y], uavs(i).threats) == 0    
                        centroids(i, 1:2) = positions(i,:);
                    end
                end
            end
        end
    else % Is control law is disable
        % centroids receive current uav positions
        centroids = positions;
    end   
    catch
         centroids = positions;
    end
end

function centroids = ComputeCentroidsRobustness(positions, controlLawsParam, uavs)
% function centroids = ComputeCentroidsRobustness(positions, controlLawsParam)
% Compute centroids to connectivity robustness improve
% Input: positions - matrix nx2 of uavs position (x,y)
%        controlLawsParam - struct Get_Parameters().ControlLawsParam();
%        uavs - matrix nx1 of struct UAV
% Output: centroidsVoronoi -  matrix nx2 of goal position (x,y)
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 19/07/2017

    if controlLawsParam.adpGainRobustness > 0
        % Get centroids for coverage improvement
        centroids = define_goalPosition(positions);
        for i = 1: size(centroids, 1)
            % Check if centroid is inside some threat
            if PointIsInsideSomeThreat(centroids(i,1:2), uavs(i).threats) ...
                dist = CalculateDistance(uavs(i).x, uavs(i).y, centroids(i,1), centroids(i,2));
                if dist < uavs(i).speed * 2 && LineIntersectsObstacle([uavs(i).x, uavs(i).y], centroids(i,1:2), uavs(i).threats) == 0
                    % If is inside, use current UAV position 
                    centroids(i, 1:2) = positions(i,:);
                elseif dist >= uavs(i).speed
                        [x,y] = ProjectPoint(uavs(i).x, uavs(i).y, centroids(i,1), centroids(i,2), uavs(i).speed*2);
                    if LineIntersectsObstacle([uavs(i).x, uavs(i).y], [x,y], uavs(i).threats) == 0    
                        centroids(i, 1:2) = positions(i,:);
                    end
                end
            end
        end
    else % Is control law is disable
        % centroids receive current uav positions
        centroids = positions;
    end   

end