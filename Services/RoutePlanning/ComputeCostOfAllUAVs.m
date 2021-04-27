function cost = ComputeCostOfAllUAVs(uavs, scenario)

% Create clone of UAVs 
clone_uavs = repmat(uavs,1);
nUAVs = clone_uavs.Count();
costs = zeros(nUAVs, 1);
% For each UAV
for i = 1: nUAVs
    % Get current UAV
    currentUAV = clone_uavs.Value(i);
    
    % Check if uav is alive and it is not 
    if (currentUAV.Alive == 0 || currentUAV.UnderMission == 0)
        continue;
    end
    
    nGoals = currentUAV.GroundForcesToVisit.Count();
    % If UAV is returning to GCS
    if (nGoals == 0)
        goal = Position3D(1,1,0);
        costs(i) = CalculateAuxiliarCost(currentUAV.Position, goal);
    else
        goal = currentUAV.GetNextGoalPosition(scenario);
        currentUAV.GroundForcesToVisit = currentUAV.GroundForcesToVisit.RemoveValueInIndex(1);
        costs(i) = CalculateAuxiliarCost(currentUAV.Position, goal);
        currentGoal = repmat(goal,1);
        for j = 1:nGoals-1
            goal = currentUAV.GetNextGoalPosition(scenario);
            currentUAV.GroundForcesToVisit = currentUAV.GroundForcesToVisit.RemoveValueInIndex(1);
            costs(i) = costs(i) + CalculateAuxiliarCost(currentGoal, goal);        
        end        
        % Add cost to return to GCS       
        costs(i) = costs(i) + CalculateAuxiliarCost(goal, Position3D(1,1,0));    
    end    
end

cost = max(costs);

end


function cost = CalculateAuxiliarCost(currentPos, goal)

global RISK_MAP;

cost = currentPos.CalculateDistancePos(goal);
return;
[currentCellX,currentCellY] = GetCorrespondingCell(RISK_MAP, currentPos.X, currentPos.Y);
[goalCellX,goalCellY] = GetCorrespondingCell(RISK_MAP, goal.X, goal.Y);
[final, x,y] = Astar(true(RISK_MAP.Properties.M,RISK_MAP.Properties.N),1-(RISK_MAP.Map),sub2ind([RISK_MAP.Properties.M,RISK_MAP.Properties.N],currentCellX,currentCellY), sub2ind([RISK_MAP.Properties.M,RISK_MAP.Properties.N],goalCellX,goalCellY));
cost = length(x);
return;
n = length(x);
cost = 0;
for i = 1: n-1
    p1 = GetCorrespondingPointInCell(RISK_MAP, x(i), y(i));
    p2 = GetCorrespondingPointInCell(RISK_MAP, x(i+1), y(i+1));
    d = p1.CalculateDistancePos(p2);
    cost = cost + d;
end
%figure;
%a_star_plot(logical(ones(RISK_MAP.Properties.M,RISK_MAP.Properties.N)),1-RISK_MAP.Map, final);

%cost = length(a);

end
