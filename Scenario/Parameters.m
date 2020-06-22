function out = Parameters()
    % UAV
    out.UAVsSpeed = 50; %m/s
    out.angleVariation = 2*pi;%spi/2; %m/s
    % GCS
    out.GCSPosition = Position3D(1,1,0);
    % GF
    out.GFsWithoutPriority = 0;
    % Threat
    out.ThreatsDetectionRange = 700; %meters
    out.ThreatsWeaponRange = 0; %meters
    % Scenario
    out.ThreatsKnownPrior = 0;
    out.ScenarioNumberOfThreats = 6;
    out.ScenarioWidth = 7500;
    out.ScenarioHeight = 5000;
    out.ScenarioDeepth = 0;
    out.ScenarioNumberOfUAVs = 3;
    out.ScenarioNumberOfGroundForces = 10;
    out.ScenarioMinDistanceBetweenThreats = 1600;
    out.ScenarioMinDistanceThreatsAndGFs = 1000;
    out.ScenarioMinDistanceThreatsAndGCS = 2000;
    out.ScenarioMinDistanceGCSAndGFs = 800;
    out.ScenarioMinDistanceGFs = 800;
end