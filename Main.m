nUAVs = 10;
nGFs = 15;
nThreats = 2;
scenario = Scenario(nUAVs, nGFs, nThreats);
%scenario = Db(scenario);
[missionTime, nGFToVisit, destroyed]= ExecuteIteration(scenario, 30, 1);
fprintf("Time = %d\nGF to visit = %d\nDestroyed UAVs = %d\n", missionTime, nGFToVisit, destroyed); 
print("Process ended");