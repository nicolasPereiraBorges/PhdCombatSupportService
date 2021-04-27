function  RunSimulation(scenario)

global VISITED_GF;
flagVideo = 1;

% Apply route planning
scenario = DVRP(scenario);

%h2 = figure('units','normalized','outerposition',[0 0 1 1]);
timestep = 3;
%cost = HeatMap(RISK_MAP.Map);
if flagVideo == 1
    video = VideoWriter('Z:\E.mp4', 'MPEG-4');
    video.FrameRate = 6;
    open(video);
end

% Start parameters
knowAreas = ones(20000,2);
maxSimulationTime = 5000;
c = 1;
%timestep = 1;
h = figure('units','normalized','outerposition',[0 0 1 1]);
 PlotScenario(scenario,1, h, knowAreas);    
scenario = DVRPOptimizer(scenario);
for i = 1: maxSimulationTime
    
    scenario = FlyNew(scenario,1);
    for j = 1: scenario.UAVs.Count
       uav = scenario.UAVs.Value(j);
       knowAreas(c,1) = uav.Position.X;
       knowAreas(c,2) = uav.Position.Y;
       c = c + 1;
    end
    if mod(i,timestep) == 0
     
        PlotScenario(scenario,i, h, knowAreas);      
        title("Simulation time = " + num2str(i));                               
        if flagVideo == 1
            im1 = getframe(h);
            writeVideo(video, im1.cdata);
        end
        pause(0.00001);
    end
%     % All Gfs are visited
%     if  sum(VISITED_GF == 0) == 0 
%         flag1 = 1;
%         for j = 1: scenario.UAVs.Count
%             uav = scenario.UAVs.Value(j);
%             if uav.UnderMission == 1
%                 flag = 0;
%                 break;
%     if StopExecution(scenario) == 1
%         return;
%     end
end
if flagVideo == 1
    close(video);
end