function riskMap = UpdateRiskMap(riskMap)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here
    
    safe = riskMap.Safe;
    noFly = riskMap.NoFly;
    CoveredArea = riskMap.CoveredArea;
    
    riskMap.Map = safe + CoveredArea - noFly;
    
end
