function riskMap = UpdateDetectionProbability(riskMap, threat)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here
 
     cx = threat.Position.X;
      cy = threat.Position.Y;
      detectionRange = threat.DetectionRange;
%     m = riskMap.Properties.M;
%     n = riskMap.Properties.N;
%     resolution = riskMap.Properties.Resolution;
    points = riskMap.Points;
    dists = sqrt( ((cx - points(:,3)).^2) + ((cy - points(:,4)).^2) );    
    % Find points inside the detection range area
    [l,~] = find(dists < detectionRange);
    
    for i = 1: length(l)
        currentCell = points(l(i),:);
        decProb = abs(0.99 - ((dists(l(i))^4) / (detectionRange^4)));                
        riskMap.NoFly(currentCell(1),currentCell(2)) = decProb;
    end
    
    riskMap = UpdateRiskMap(riskMap);
end
