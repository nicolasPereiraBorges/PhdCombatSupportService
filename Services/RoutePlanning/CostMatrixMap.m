function costMatrix = CostMatrixMap(scenario)

global AVAILABLE_UAVS;
global VISITED_GF;

[uavsId,~] = find(AVAILABLE_UAVS == 1);
[gfsId, ~] = find(VISITED_GF == 0);

nUAVs = length(uavsId);
nGFs = length(gfsId);


% Calculate Cost Between 
keys = cell(1,1000);
values = cell(1,1000);
currentIndex = 1;

% UAV to GFs
for i = 1:nUAVs
   uav = scenario.UAVs.Value(uavsId(i));   
   for j = 1: nGFs
       % UAV - > GFs
      gf = gfsId(j);
      gfPos = scenario.GroundForces.Value(gf).Position;
      cost = CalculateCostAStar(uav.Position, gfPos);
      keys{currentIndex} = sprintf('UAV%dGF%d', uav.Id, gf);
      values{currentIndex} = cost;
      currentIndex = currentIndex + 1;      
   end
   % UAV -> GCS
   cost = CalculateCostAStar(uav.Position, Position3D(1,1,0));
   keys{currentIndex} = sprintf('UAV%dGCS', uav.Id);
   values{currentIndex} = cost;
   currentIndex = currentIndex + 1;      
end

% GFs to GFs
for i = 1:nGFs
   gfBefore = gfsId(i);
   gfBeforePos = scenario.GroundForces.Value(gfBefore).Position;
   for j = i + 1: nGFs    
          gfAfter = gfsId(j);          
          gfAfterPos = scenario.GroundForces.Value(gfAfter).Position;
          cost = CalculateCostAStar(gfBeforePos, gfAfterPos);
          keys{currentIndex} = sprintf('GF%dGF%d', gfBefore, gfAfter);
          values{currentIndex} = cost;
          currentIndex = currentIndex + 1;      
          keys{currentIndex} = sprintf('GF%dGF%d', gfAfter, gfBefore);
          values{currentIndex} = cost;
          currentIndex = currentIndex + 1;            
   end   
   % GFs to GCS
   cost = CalculateCostAStar(gfBeforePos, Position3D(1,1,0));
   keys{currentIndex} = sprintf('GF%dGCS', gfBefore);
   values{currentIndex} = cost;
   currentIndex = currentIndex + 1;      
end

keys = keys(1:currentIndex-1);
values = values(1:currentIndex-1);
costMatrix = containers.Map(keys, values);

end



function cost = CalculateCostAStar(currentPos, goal)

global RISK_MAP;

[currentCellX,currentCellY] = GetCorrespondingCell(RISK_MAP, currentPos.X, currentPos.Y);
[goalCellX,goalCellY] = GetCorrespondingCell(RISK_MAP, goal.X, goal.Y);
[final, ~,~] = Astar(true(RISK_MAP.Properties.M,RISK_MAP.Properties.N),1-(RISK_MAP.Map),sub2ind([RISK_MAP.Properties.M,RISK_MAP.Properties.N],currentCellX,currentCellY), sub2ind([RISK_MAP.Properties.M,RISK_MAP.Properties.N],goalCellX,goalCellY));
cost = (length(final)*100);% + (currentPos.CalculateDistancePos(goal)*0.1);

end
