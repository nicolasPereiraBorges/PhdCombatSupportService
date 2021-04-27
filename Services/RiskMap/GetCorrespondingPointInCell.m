function point = GetCorrespondingPointInCell(riskMap, i,j)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here
    
    
%     m = riskMap.Properties.M;
%     n = riskMap.Properties.N;
%     resolution = riskMap.Properties.Resolution;
    
    points = riskMap.Points;
    
    [l,~] = find(points(:,1)==i & points(:,2) == j);
    
    x = points(l,3);
    y = points(l,4);
    point = Position3D(x,y);
    %x = ((i+0.5)*resolution)-(((n+1)*resolution)/2);
    %y = ((j+0.5)*resolution)-(((m+1)*resolution)/2);
    
    
end

