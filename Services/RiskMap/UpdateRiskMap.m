function riskMap = UpdateRiskMap(riskMap)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here
    
    safe = riskMap.Safe;
    noFly = riskMap.NoFly;
    CoveredArea = riskMap.CoveredArea;
    
    riskMap.Map = noFly.*-1;
    riskMap.Map(riskMap.Map >= 0) = safe(riskMap.Map >= 0);
    %riskMap.Map = safe + CoveredArea - noFly;
    
end
