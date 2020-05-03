function out = Parameters()
    % UAV
    out.UAVsSpeed = 30; %m/s
    % GCS
    out.GCSPosition = Position3D(1,1,0);
    % GF
    out.GFsWithoutPriority = 0;
    % Threat
    out.ThreatsDetectionRange = 400; %meters
    out.ThreatsWeaponRange = 0; %meters
    % Scenario
    out.ScenarioNumberOfThreats = 3;
    out.ScenarioWidth = 7500;
    out.ScenarioHeight = 5000;
    out.ScenarioDeepth = 0;
    out.ScenarioNumberOfUAVs = 5;
    out.ScenarioNumberOfGroundForces = 5;
    out.ScenarioMinDistanceBetweenThreats = 600;
    out.ScenarioMinDistanceThreatsAndGFs = 600;
    out.ScenarioMinDistanceThreatsAndGCS = 800;
    out.ScenarioMinDistanceGCSAndGFs = 800;
    out.ScenarioMinDistanceGFs = 800;
end