function out = Parameters()
    % Simulation
    out.ReplanOptimizer = 0;
    out.RiskMap =  1;
    % UAV
    out.UAVsSpeed = 50; %m/s
    out.angleVariation = 2*pi;%spi/2; %m/s
    % GCS
    out.GCSPosition = Position3D(1,1,0);
    % GF
    out.GFsWithoutPriority = 0;
    % Threat
    out.ThreatsDetectionRange = 600; %meters
    out.ThreatsWeaponRange = 0; %meters
    %out.ExposureTimeThreshold = 4;
    %out.ExposureTimeThreshold = 10;
    out.ExposureTimeThreshold = 24444;
    %out.DeathProbability = 0.25;
    out.DeathProbability = 0.00;
    %out.DeathProbability = 0.0;
    % Scenario
    out.Plot = 0;
    out.StealthFlag = 0;
    out.ThreatsKnownPrior = 0;
    out.ScenarioNumberOfThreats = 8;
    out.ScenarioWidth = 7500;
    out.ScenarioHeight = 5000;
    out.ScenarioWidth = 8500;
    out.ScenarioHeight = 6000;
    out.ScenarioDeepth = 0;
    out.ScenarioNumberOfUAVs = 10;
    out.ScenarioNumberOfGroundForces = 15;
    out.ScenarioMinDistanceBetweenThreats = 700;
    out.ScenarioMinDistanceThreatsAndGFs = 700;
    out.ScenarioMinDistanceThreatsAndGCS = 1000;
    out.ScenarioMinDistanceGCSAndGFs = 400;
    out.ScenarioMinDistanceGFs = 200;
end