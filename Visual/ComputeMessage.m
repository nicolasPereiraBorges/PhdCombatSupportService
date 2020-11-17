function message = ComputeMessage(id, uavId, param)

switch id
    case {1}
        message = sprintf("%s -> GCS --- UAV:PositionUpdate", uavId);
    case {2}
        message = sprintf("%s -> GCS --- UAV:ThreatInformation (%s)", uavId, param);
    case {3}
        message = sprintf("%s -> GCS --- UAV:GFVisited (%d)", uavId, param);
    case {4}
        message = sprintf("GCS -> %s --- GCS:ThreatListUpdate (%s)", uavId, param);
    case {5}
        message = sprintf("GCS -> %s --- GCS:SpecificThreatUpdate (%s)", uavId, param);
    case {6}
        message = sprintf("GCS -> %s --- GCS:NewRoutes (%s)", uavId, param);
    otherwise
        message = "";
        
end
end