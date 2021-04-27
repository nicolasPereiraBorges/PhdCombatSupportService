function [newGoal] = GetNextGoalUAVRiskMap(uav,goal)
global RISK_MAP;
[currentCellX,currentCellY] = GetCorrespondingCell(RISK_MAP, uav.Position.X, uav.Position.Y);
[goalCellX,goalCellY] = GetCorrespondingCell(RISK_MAP, goal.X, goal.Y);
[final, x,y] = Astar(true(RISK_MAP.Properties.M,RISK_MAP.Properties.N),1-(RISK_MAP.Map),sub2ind([RISK_MAP.Properties.M,RISK_MAP.Properties.N],currentCellX,currentCellY), sub2ind([RISK_MAP.Properties.M,RISK_MAP.Properties.N],goalCellX,goalCellY));
%a_star_plot(logical(ones(RISK_MAP.Properties.M,RISK_MAP.Properties.N)),1-RISK_MAP.Map, final);
x = x(end-1:-1:2);
y = y(end-1:-1:2);
n = length(x);
newGoal = List();
if n > 1
    for i = 1:n
        newGoal = newGoal.AddLast(GetCorrespondingPointInCell(RISK_MAP, x(i), y(i)));
    end
end
end
