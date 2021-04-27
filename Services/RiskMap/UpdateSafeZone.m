function riskMap = UpdateSafeZone(riskMap, i,j)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here
    
    
    [x,y] = GetCorrespondingCell(riskMap, i,j);
    riskMap.Safe(x,y) = riskMap.Safe(x,y) + 0.3;
    %riskMap.CoveredArea(x,y) = -100;
     if (riskMap.Safe(x,y) > 1)
         riskMap.Safe(x,y) = 1;
     end
    riskMap = UpdateRiskMap(riskMap);
end
