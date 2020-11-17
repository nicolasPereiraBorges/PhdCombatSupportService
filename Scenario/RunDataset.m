function RunDataset(rootFolder, startDataset,endDataset, stealth)
%CREATEDATASET Summary of this function goes here
%   Detailed explanation goes here


nUAVs = [1,2,5,8,10];
nThreats = [0,2,5,8,12];
nGFs = [1,5,10,15,20];
%nGFs = [10,15,20];


% 
% % nThreats = 12;
%   nGFs = 15;
%    nUAVs = 1;
% 
%   nThreats = 8;
nDataset = endDataset - startDataset + 1;


aux = sprintf("_%d_%d_", startDataset,endDataset);
if stealth == 1
    fileID = fopen(rootFolder + sprintf("FINAL_saidaComStealth%s.txt", aux),'w');
else
    fileID = fopen(rootFolder + sprintf("FINAL_saidaSemStealth%s.txt", aux),'w');
end

fprintf(fileID,"UAVs;GFs;Threats;i;Concluded;Score_Time;Score_GF;Score_Dest;Stealth\n");

for threats=nThreats
    for j = nUAVs
        for gfs=nGFs
            endFolder = rootFolder + "\\" + sprintf("T=%d\\G=%d\\",threats,gfs);
            gfToVisit = zeros(nDataset,1);
            avTime = zeros(nDataset,1);
            concluded = zeros(nDataset,1);
            destroyedUAVs = zeros(nDataset,1);
            
            parfor i = startDataset:endDataset
            %for i = 1:nDataset
                s = sprintf("UAVs:%d---GFs:%d--Threats:%d--I=%d",j, gfs, threats, i);
                fprintf ("Running %s\n", s);
                nRep = 1;
                
                gfToVisitAUX = zeros(nRep,1);
                avTimeAUX = zeros(nRep,1);
                concludedAUX = zeros(nRep,1);
                destroyedUAVsAUX = zeros(nRep,1);                
                for kk = 1: nRep
                    [t,g,dest] = Process(endFolder + sprintf("%d.mat",i),j, stealth);                    
                    if (g == 0)
                        avTimeAUX(kk) = t;
                        concludedAUX(kk) = 1;                        
                    end
                    destroyedUAVsAUX(kk) = dest;
                    gfToVisitAUX(kk) = g;
                end              
                
                gfToVisit(i) = mean(gfToVisitAUX);
                concluded(i) = mean(concludedAUX);
                destroyedUAVs(i) = mean(destroyedUAVsAUX);
                if (sum(concludedAUX)) > 0
                    avTime(i) = sum(avTimeAUX) / sum(concludedAUX);
                else
                    avTime(i) = -1;
                end
            end                       
            for i = startDataset:endDataset
                s = sprintf("%d;%d;%d;%d;%d;%d;%d;%d;%d\n",j, gfs, threats, i, concluded(i), avTime(i), gfToVisit(i),destroyedUAVs(i),stealth);
                fprintf(fileID,s);
            end
        end
         
        
    end
end
fclose(fileID);

end

function [time, gf, destroyed] = Process(path, nUAVs, stealth)
load(path);
scenario.UAVs =  CreateUavs(scenario.GCS, nUAVs);
tic;
[time, gf, destroyed] = ExecuteIteration(scenario, 2, stealth);
toc;
clear scenario;
end
