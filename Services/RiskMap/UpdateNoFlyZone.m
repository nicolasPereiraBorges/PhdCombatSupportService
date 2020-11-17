function riskMap = UpdateNoFlyZone(riskMap, i,j)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here
    
    %[x,y] = GetCorrespondingCell(riskMap, i,j);
    riskMap.NoFly(i,j) = 100;
    %riskMap.CoveredArea(i,j) = 0;
    %riskMap.Safe(i,j) = 0;
    
end
