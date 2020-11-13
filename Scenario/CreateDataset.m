function CreateDataset(rootFolder, nDataset)
%CREATEDATASET Summary of this function goes here
%   Detailed explanation goes here
c = clock();
year = c(1);
day = c(3);
month = c(2);
folder = rootFolder + "Dataset" + "_"+ num2str(day) + "_" + ...
    num2str(month) + "_" + num2str(year);

if ~exist(folder, 'dir')
    mkdir(folder)
end

 nThreats = [0, 2, 5, 8, 12];
 nUAVs = [1, 2, 5];
 nGFs = [1,4,5,5,5];
%nThreats = 8;
for threats=nThreats    
    parfor i = 1:nDataset
    scenario = Scenario(1,1,threats);    
    refThreats = scenario.Threats;
    gfRef = List();
    gfId = 1;
    s = 0;
    for gfs=nGFs   
        s = s+gfs;
        if gfs == 1
            endFolder = folder + "\\" + sprintf("T=%d\\G=%d\\",threats,s);                       
        else
            endFolder = folder + "\\" + sprintf("T=%d\\G=%d\\",threats,s);                       
        end
        if ~exist(endFolder, 'dir')
            mkdir(endFolder)
        end
        for j = 1: gfs
            point = GenerateValidPositionGF(refThreats, scenario.GCS, gfRef);
            gf = GroundForce();
            gf.Id = gfId;    
            gfId = gfId + 1;
            gf.Position = point;
            gfRef = gfRef.AddLast(gf);
            scenario.GroundForces = gfRef;
            
            process(endFolder + num2str(i) + ".mat", scenario);
        end
    end
        
    end
end
end

function process(path, scenario)
save(path, "scenario");
end

function pos = GenerateValidPositionGF(threats, gcs, groundForces)

parameters = Parameters();
minDistanceToGCS = parameters.ScenarioMinDistanceThreatsAndGCS;
minDistanceToThreats = parameters.ScenarioMinDistanceBetweenThreats;
minDistanceToGFS = parameters.ScenarioMinDistanceThreatsAndGFs;
xlim = [0, parameters.ScenarioWidth];
ylim = [0, parameters.ScenarioHeight];
zlim = [0, parameters.ScenarioDeepth];

valid = 0;
while(valid == 0)
    valid = 1;
    pos = GenerateRandomPosition(xlim, ylim, zlim);
    
    % Distance to GCS
    if (CalculateDistance(pos, gcs.Position) < minDistanceToGCS)
        valid = 0;
        continue;
    end
    % Distance to GFs
    for i = 1:groundForces.Count()
        if (CalculateDistance(pos, groundForces.Value(i).Position) < minDistanceToGFS)
            valid = 0;            
        end
    end            
    % Distance to other threats
    for i = 1:threats.Count()
        d = CalculateDistance(pos, threats.Value(i).Position);
        if (d < minDistanceToThreats)
            valid = 0;            
        end
    end
      
end
end



