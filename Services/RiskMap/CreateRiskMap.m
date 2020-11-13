function risk = CreateRiskMap(height, width, resolution)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here
    
    
    m = ceil(height/resolution);
    n = ceil(width/resolution);
        
    safeZoneRisk = CreateRiskMapLayer(m,n);
    noFlyZoneRisk = CreateRiskMapLayer(m,n);
    coveredArea = CreateRiskMapLayer(m,n);
    map = CreateRiskMapLayer(m,n);    
    points = CreateRefPoints(m,n,resolution);
    
    
    properties = struct('M', m,...
                  'N', n, ...
                  'Resolution', resolution);
    
    risk = struct('Map', map,...
                  'Safe', safeZoneRisk,...
                  'NoFly', noFlyZoneRisk, ...
                  'CoveredArea', coveredArea,...
                  'Properties', properties, ...
                  'Points', points);
    
    risk = UpdateSafeZone(risk,1,1);
    %risk = UpdateNoFlyZone(risk,height,width);
    risk = UpdateRiskMap(risk);
end

function points = CreateRefPoints(m,n,resolution)

points = zeros(m*n, 4);
count = 1;
for i = 1:m
   for j = 1:n
    points(count,1)= i;
    points(count,2)= j;
    points(count,3)= i*resolution;
    points(count,4)= j*resolution;
    count = count + 1;
   end
end

end
function risk = CreateRiskMapLayer(m, n)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here

    risk = zeros(m,n);
end