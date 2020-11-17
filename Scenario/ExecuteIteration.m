function [missionTime, gfVisited, destroyedUAVs]= ExecuteIteration(scenario, timestep, stealth)

global REPORT_EXPOSURE_TIME;
global LOG;
LOG = "";
global TIME;
TIME = 0;
global EXPOSURE_TIME;
global AVAILABLE_UAVS;
global VISITED_GF;
global COST_MAP;


param = Parameters();
REPORT_EXPOSURE_TIME = 0;
scenario = DVRP(scenario);
destroyedUAVs = -1;
gfVisited = 0;
%PlotScenario(scenario,0, LOG);
%waitforbuttonpress;
nUAVs = scenario.UAVs.Count();    
AVAILABLE_UAVS = ones(nUAVs,1);
% VISITED_GF = zeros(

COST_MAP = CreateCostMap(param.Scenarioheight, param.ScenarioWidth);

for missionTime = 1: 15000
    if mod(missionTime,timestep) == 200
         fprintf("I = %d\n", missionTime);
     end
    LOG = "";
    TIME = missionTime;  
    try
    scenario = Fly(scenario, stealth);
    catch a
        a = 1;
    end
    
      if param.Plot == 1 && mod(missionTime,timestep) == 0
         PlotScenario(scenario,missionTime, LOG);
          waitforbuttonpress;
      end
    %fprintf("%s\n", LOG);
    if StopExecution(scenario) == 1 || sum(VISITED_GF==0) == 0
        %fprintf("EXPOSURE TIME = %d\n", REPORT_EXPOSURE_TIME);
        gfVisited = sum(VISITED_GF == 0);
        destroyedUAVs = 0;
        nUAVs = scenario.UAVs.Count();
        for i = 1: nUAVs
        uav = scenario.UAVs.Value(i);
        if uav.Alive == 0
            destroyedUAVs = destroyedUAVs  +  1;
        end
        end
        return;
    end
end

gfVisited = sum(VISITED_GF == 0);

end


function stop = StopExecution(scenario)
global VISITED_GF;

    nUAVs = scenario.UAVs.Count();
    for i = 1: nUAVs
        uav = scenario.UAVs.Value(i);
        
        if uav.GroundForcesToVisit.Count() > 0 || sum(VISITED_GF==0) > 0
        %if uav.UnderMission == 1
            stop = 0;
            return;
        end
    end

    stop = 1;
end

function scenario = DVRP(scenario)
global EXPOSURE_TIME;
global AVAILABLE_UAVS;
global VISITED_GF;

param = Parameters();
nGF = scenario.GroundForces.Count();
nUAVs = scenario.UAVs.Count();
EXPOSURE_TIME = zeros(nUAVs,1);
AVAILABLE_UAVS = ones(nUAVs,1);
VISITED_GF = zeros(nGF,1);

currentUAV = 1;
for i = 1: nGF
    if (currentUAV > nUAVs)
        currentUAV = 1;
    end    
    
    uav = scenario.UAVs.Value(currentUAV);
    if param.ThreatsKnownPrior == 1
        uav.Threats = scenario.Threats;        
        for j = 1:scenario.Threats.Count()
            t = scenario.Threats.Value(j);
            t.Detected = 1;
            scenario.Threats = scenario.Threats.ModifyValueInIndex(j, t);        
        end

    end
    uav.UnderMission = 1;
    uav.GroundForcesToVisit = uav.GroundForcesToVisit.AddLast(scenario.GroundForces.Value(i).Id);
    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(currentUAV, uav);
    currentUAV = currentUAV + 1;
end

end

function scenario = Replan_Remove_DVRP(scenario, uav_id)
global AVAILABLE_UAVS;

nUAVs = scenario.UAVs.Count();
% Exposed UAV
uav = scenario.UAVs.Value(uav_id);
gfs = uav.GroundForcesToVisit;
uav.GroundForcesToVisit = List();
scenario.UAVs = scenario.UAVs.ModifyValueInIndex(uav_id, uav);
nGF = gfs.Count();

if (sum(AVAILABLE_UAVS) == 0)
    return;
end

currentUAV = 1;
for i = 1: nGF
    
    r = randi([1,nUAVs]);
    while(AVAILABLE_UAVS(r) == 0)
        r = randi([1,nUAVs]);
    end
        
    uav = scenario.UAVs.Value(r);
    uav.UnderMission = 1;
    uav.GroundForcesToVisit = uav.GroundForcesToVisit.AddLast(gfs.Value(i));
    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(r, uav);    
end

end

function scenario = Replan_Add(scenario)
global AVAILABLE_UAVS;
global VISITED_GF;

if (sum(AVAILABLE_UAVS) == 0)
    return;
end

gfs = 1:length(VISITED_GF);
[availables,~] = find(AVAILABLE_UAVS == 1);
n_availables = length(availables);
nGF = length(gfs);
gfsAvailables = ones(nGF,1);
gfsAvailables(VISITED_GF == 1) = 0;
nInt = floor(sum(gfsAvailables==1) / n_availables);

for i = 1: n_availables
     uav = scenario.UAVs.Value(availables(i));
     uav.GroundForcesToVisit = List(); 
     %uav.FlightPath = FlightPath();
     uav.FlightPath = uav.FlightPath.Trucante(50);
         
     scenario.UAVs = scenario.UAVs.ModifyValueInIndex(availables(i), uav); 
     for j = 1: nInt        
        r = randi([1,nGF]);
        while(gfsAvailables(r) == 0)
            r = randi([1,nGF]);
        end    
        gfsAvailables(r) = 0;        
        uav = scenario.UAVs.Value(availables(i));
        uav.UnderMission = 1;
        uav.GroundForcesToVisit = uav.GroundForcesToVisit.AddLast(gfs(r));
        scenario.UAVs = scenario.UAVs.ModifyValueInIndex(availables(i), uav); 
     end
end

l = List();
for i = 1:nGF
   if  gfsAvailables(i) == 1
       l = l.AddLast(gfs(i));
   end
end
gfs = l;
nGF = l.Count();
% Exposed UAV
nUAVs = length(AVAILABLE_UAVS);
for i = 1: nGF
    
    r = randi([1,nUAVs]);
    while(AVAILABLE_UAVS(r) == 0)
        r = randi([1,nUAVs]);
    end
    %uav.FlightPath = uav.FlightPath.Trucante(50);    
    uav = scenario.UAVs.Value(r);
    uav.UnderMission = 1;
    uav.GroundForcesToVisit = uav.GroundForcesToVisit.AddLast(gfs.Value(i));
    scenario.UAVs = scenario.UAVs.ModifyValueInIndex(r, uav);    
end

end

function scenario = Fly(scenario, stealth)
global VISITED_GF;
global EXPOSURE_TIME;
global AVAILABLE_UAVS;
global REPORT_EXPOSURE_TIME;
global COST_MAP;

param = Parameters();

nUAVs = scenario.UAVs.Count();

% Para cada UAV
for i = 1: nUAVs    
    % Captura UAV
    uav = scenario.UAVs.Value(i);
    
    if uav.GroundForcesToVisit.Count() == 0 && scenario.GCS.Position.CalculateDistancePos(uav.Position) < 100
        uav.UnderMission = 0;
        scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav); 
    end
    if (uav.Alive == 0 || uav.UnderMission == 0)
        continue;
    end
    %GenerateLog(ComputeMessage(1,num2str(uav.Id)));
    lastExp = uav.Exposed;
    [scenario, ~, ~] = DetectThreats(scenario, i, stealth);    
    
    uav = scenario.UAVs.Value(i);
    if (uav.Exposed == 1)
        REPORT_EXPOSURE_TIME = REPORT_EXPOSURE_TIME + 1;
        AVAILABLE_UAVS(i) = 0;               
        EXPOSURE_TIME(i) = EXPOSURE_TIME(i) + 1;
        if stealth  == 1
            if (EXPOSURE_TIME(i) == param.ExposureTimeThreshold) ...
            || (uav.Alive==0)                
                scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);        
                tt = scenario.Threats;
                scenario = Replan_Remove_DVRP(scenario, i);               
                scenario.Threats = tt;
                return;
                %uav = scenario.UAVs.Value(i);
            end
        else
           if (uav.Alive==0)               
               scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav);  
               scenario = Replan_Remove_DVRP(scenario, i);
                
                return;
                %uav = scenario.UAVs.Value(i);
            end 
        end

        if (uav.Alive == 0)
            continue;
        end
    
        
    else
        AVAILABLE_UAVS(i) = 1;
        EXPOSURE_TIME(i) = 0;
        if (lastExp == 1 &&  stealth  == 1 && uav.GroundForcesToVisit.Count()==0)            
            scenario = Replan_Add(scenario);
            return;
        else
%             if (uav.FlightPath.Count() > 50)
%                  goal = uav.GetNextGoalPosition(scenario);        
%                  dist = uav.FlightPath.Last().CalculateDistancePos(goal);    
%                  if (dist > 2000)
%                      uav.FlightPath = FlightPath();
%                  end
%             end
            %uav = scenario.UAVs.Value(i);
        end        
    end
    
    flightPath = uav.FlightPath;
    if flightPath.Count() > 0        
        uav.Position = flightPath.First();
        uav.FlightPath.Positions = uav.FlightPath.Positions.RemoveValueInIndex(1);
        uav = uav.UpdatePathHistory();
        %pat = uav.FlightPath;
        %points = ReducePath(pat,c_threats);
        %points = ApplyFixedSpace(points, uav.Speed); 
        %uav.FlightPath = uav.FlightPath.UpdatePosGivenArray(points);
         %uav = ApplyRRT_InC(uav, goal);     
    else
        goal = uav.GetNextGoalPosition(scenario);        
            dist = uav.Position.CalculateDistancePos(goal);            

            if dist < 210                
                idGroundForce = uav.GroundForcesToVisit.First();
                if ~isempty(idGroundForce)
                    gf = scenario.GroundForces.Value(idGroundForce);
                    gf.Visited = 1;
                    VISITED_GF(gf.Id) = 1;
                    scenario.GroundForces = scenario.GroundForces.ModifyValueInIndex(idGroundForce, gf);                 
                    uav.GroundForcesToVisit = uav.GroundForcesToVisit.RemoveValueInIndex(1);
                end                
                uav.FlightPath = FlightPath();
            else
                %goal = uav.GetNextGoalPosition(scenario);
                uav = ApplyRRT_InC(uav, goal);                
            end
    end
    COST_MAP(floor(uav.Position.X), floor(uav.Position.Y)) = 0;
   
   scenario.UAVs = scenario.UAVs.ModifyValueInIndex(i, uav); 
end
end


function GenerateLog(message)
return;
global LOG;
global TIME;

LOG = "Time: " + num2str(TIME) +  " sec -> " + message + "\n"+LOG; 
if length(LOG) > 500    
    LOG = LOG(1:500);
end

end