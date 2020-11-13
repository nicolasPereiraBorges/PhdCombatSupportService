function uav = CalculateGoalToLeaveThreat(uav, detectedThreat)
 
    [p1, p2] = GetPerpendicularPoints(uav, detectedThreat);
    p11 = Position3D(p1(1), p1(2));
    p22 = Position3D(p2(1), p2(2));
    d1 = p11.CalculateDistancePos(uav.Position);
    d2 = p22.CalculateDistancePos(uav.Position);
    if (d1 < d2)       
            goal = p1;        
    else       
            goal = p2;        
    end
    uav.FlightPath = FlightPath();
    points = [[uav.Position.X, uav.Position.Y]; goal];
    points = ApplyFixedSpace(points, uav.Speed); 
    uav.FlightPath = uav.FlightPath.UpdatePosGivenArray(points);                

end