function [x,y] = GetCorrespondingCellThreat(riskMap, threat)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here
    
    
%     m = riskMap.Properties.M;
%     n = riskMap.Properties.N;
%     resolution = riskMap.Properties.Resolution;
    points = riskMap.Points;
    dists = sqrt( ((threat.Position.X - points(:,3)).^2) + ((threat.Position.Y - points(:,4)).^2) );
    [l,~] = find(dists <= min(threat.DetectionRange));    
    x = points(l,1);
    y = points(l,2);
    %x = ((i+0.5)*resolution)-(((n+1)*resolution)/2);
    %y = ((j+0.5)*resolution)-(((m+1)*resolution)/2);
    
    
end

