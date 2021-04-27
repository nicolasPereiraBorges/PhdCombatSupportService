
function scenario = ReOrderDVRP(scenario)
%return;
global RISK_MAP;
param = Parameters();
if param.ReplanOptimizer == 1    
    nUAVs = scenario.UAVs.Count();
    for i = 1: nUAVs
        uav = scenario.UAVs.Value(i);
        [currentCellX,currentCellY] = GetCorrespondingCell(RISK_MAP, uav.Position.X, uav.Position.Y);
        currentCell = sub2ind([RISK_MAP.Properties.M,RISK_MAP.Properties.N],currentCellX,currentCellY);
        [gfsLabel, gfsPosition] = GetGFsPosition(scenario, uav);
        gfs = unique(perms(gfsPosition),'rows');
        
        if size(gfs,1) > 10
            indexes = randperm(size(gfs,1),10);
        else
            indexes = 1:size(gfs,1);
        end
        
        bestCost = inf;
        bestCostLabel = -1;
        for j = 1:size(indexes,2)
            gf = [currentCell, gfs(indexes(j),:), 1];
            cost = ComputeCostOfOrder(gf);
            if cost < bestCost
                bestCost = cost;
                bestCostLabel = indexes(j);
            end
        end
        
        currentCost = ComputeCostOfOrder([currentCell, gfsPosition,1]);
        if currentCost <= bestCost
            gfs(bestCostLabel,:) = gfsPosition;
       
        end
    gfsList = List();
    bestCenter = gfs(bestCostLabel,:);
    for k = 1: length(gfsLabel)
        [~,c] = find(gfsPosition == bestCenter(k));
        gfsList = gfsList.AddLast(gfsLabel(c));
    end
    uav.GroundForcesToVisit = gfsList;
    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);    
        
    end
    
end
end



function [gfsLabel, gfsPosition] = GetGFsPosition(scenario, uav)
global RISK_MAP;
cloneUav = repmat(uav,1);

gfsPosition = zeros(1, uav.GroundForcesToVisit.Count());
gfsLabel = zeros(1, uav.GroundForcesToVisit.Count());

for i = 1: uav.GroundForcesToVisit.Count()
    gfsLabel(i) = uav.GroundForcesToVisit.Value(i);
    goal = cloneUav.GetNextGoalPosition(scenario);
    cloneUav.GroundForcesToVisit = cloneUav.GroundForcesToVisit.RemoveValueInIndex(1);
    [xCell,yCell] = GetCorrespondingCell(RISK_MAP,goal.X,goal.Y);
    goalCell = sub2ind([RISK_MAP.Properties.M,RISK_MAP.Properties.N],xCell,yCell);
    gfsPosition(i) = goalCell;
end

end




function cost = ComputeCostOfOrder(gfs)
global RISK_MAP;

cost = 0;
for i = 1: length(gfs)-1
    currentCell = gfs(i);
    next = gfs(i+1);
    [~, x,~] = Astar(true(RISK_MAP.Properties.M,RISK_MAP.Properties.N),1-(RISK_MAP.Map),currentCell, next);
    cost = cost + length(x);
end
end
