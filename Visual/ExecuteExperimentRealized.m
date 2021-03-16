function ExecuteExperimentRealized()

global LOG;
LOG = "";
global TIME;
TIME = 0;
global EXPOSURE_TIME;
global AVAILABLE_UAVS;
global VISITED_GF;
global RISK_MAP;

%load('Z:\Doutorado\Source\Dataset_22_7_2020\T=5\G=10\14.mat');
%load('Z:\Doutorado\Source\Dataset_22_7_2020\T=5\G=20\21.mat');
load('Z:\Doutorado\Source\Dataset_22_7_2020\T=12\G=10\6.mat');
%load('Z:\Doutorado\Source\Dataset_22_7_2020\T=12\G=10\1.mat');
scenario.UAVs =  CreateUavs(scenario.GCS, 3);
param = Parameters();

%COST_MAP = CreateCostMap(param.ScenarioHeight, param.ScenarioWidth);
RISK_MAP = CreateRiskMap(param.ScenarioHeight, param.ScenarioWidth, 200);
%NO_FLY_ZONE = repmat(COST_MAP,1);
%SAFETY_FLY_ZONE = repmat(COST_MAP,1);
%scenario = Scenario(5,15,5);
scenario = DVRP(scenario);
%print("Process ended");

h = figure('units','normalized','outerposition',[0 0 1 1]);
%h2 = figure('units','normalized','outerposition',[0 0 1 1]);
timestep = 2;
%cost = HeatMap(RISK_MAP.Map);
video = VideoWriter('Z:\Teste2511_risk_normal_8fps.mp4', 'MPEG-4');
video.FrameRate = 8;
open(video);
knowAreas = zeros(20000,2)+1;
c = 1;
for i = 1: 5000
    LOG = "";
    TIME = i;
    scenario = Fly(scenario,1);
    for j = 1: scenario.UAVs.Count
       uav = scenario.UAVs.Value(j);
       knowAreas(c,1) = uav.Position.X;
       knowAreas(c,2) = uav.Position.Y;
       c = c + 1;
    end
%     if mod(i, 100)==0
%         scenario = ReOrderDVRP(scenario);
%     end
    if mod(i,timestep) == 0
        %RISK_MAP = UpdateUnkwonwAreaMap(RISK_MAP);
        %RISK_MAP = UpdateRiskMap(RISK_MAP);
        PlotScenario(scenario,i, h, knowAreas);
        %xlim([0, param.ScenarioWidth]);
        %ylim([0, param.ScenarioHeight]);
        title("Simulation time = " + num2str(i));
        %set(0, 'CurrentFigure', h2)
        %cla reset
        %UpdateCostMapThreats(scenario);
        %COST_MAP = NO_FLY_ZONE + SAFETY_FLY_ZONE;
        %cost.set('Data', RISK_MAP.Map');
        %close(h2);
        %clear h2;
        %h2 = figure('units','normalized','outerposition',[0 0 1 1]);
        %plot(cost, h2);
        factor = 1000;
        %Clear screen and plot base points
        
        %frame = export_fig(gcf, 'n', '-eps');
        title("Simulation time = " + num2str(i));
                 im1 = getframe(h);
        %         im2 = getframe(h2);
        %          im = [im1.cdata; im2.cdata];
        %figure;
        %imshow(im);
        try
            %              imwrite(im1.cdata, "Z:\Teste1310\Path\" + num2str(i) + ".png")
            %              imwrite(im2.cdata, "Z:\Teste1310\Map\" + num2str(i) + ".png")
            %              imwrite(im, "Z:\Teste1310\Full\" + num2str(i) + ".png")
            title("Simulation time = " + num2str(i));
            im1 = getframe(h);
            writeVideo(video, im1.cdata);
        catch aaaaaa
        end
        
        %writeVideo(video, im2.cdata);
        pause(0.001);
    end
    fprintf("%s\n", LOG);
    %if StopExecution(scenario) == 1
    %    return;
    %end
end
close(video);
% close(video2);
end

