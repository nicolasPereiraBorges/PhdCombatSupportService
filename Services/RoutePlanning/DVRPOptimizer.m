function scenario = DVRPOptimizer(scenario)

global VISITED_GF;
global MAX_TIME;
global MIN_TIME;
global USE_MAX_TIME ;

USE_MAX_TIME = 1;

global BEST_SOLUTIONS;
global BEST_FITNESS;
global INDEX_BEST;
global TIME_OF_FLIGHT;


array1 = ConvertUavsToCromossome(scenario.UAVs);

BEST_SOLUTIONS = zeros(15, length(array1));
BEST_FITNESS = zeros(15,1);
TIME_OF_FLIGHT = repmat(BEST_FITNESS,1);
INDEX_BEST = 0;

 if sum(array1>0) <=1
     return;
 end
 
[array,~] = find(VISITED_GF == 0);
array = array';
first = 1;
for i = 1: scenario.UAVs.Count
    uav = scenario.UAVs.Value(i);
    %if uav.Alive == 1 && uav.Exposed == 0
       if first == 1
           first = 0;
       else
        array = [array, -1];
       end
   %end
end
    
if sum(sort(array) ~= sort(array)) == 0
    array = array1;
end


BEST_SOLUTIONS = zeros(15, length(array));

m = CostMatrixMap(scenario);

popSize = 200;
nGenerations = 500;
minDist = 0;

f = FitnessMap(array, m, scenario);
%f = Fitness(array, scenario);
[pop, totalFlightTime, timeOfVisitingPop] = CreatPopulation(array, popSize, scenario, m);

MAX_TIME = max(totalFlightTime);
MIN_TIME = min(totalFlightTime);

if USE_MAX_TIME == 1
    fitness = CalculateAreaTimePopulation(pop, timeOfVisitingPop, totalFlightTime);
else
    fitness = totalFlightTime;
end

for i = 1:nGenerations
    [pop, fitness, timeOfVisitingPop, totalFlightTime] = SortPop(pop, fitness, timeOfVisitingPop, totalFlightTime);    
    k = popSize;
    flagChangeTime = 0;
    for j = 1: 3
        % Selection
        selected = TournamentSelection(pop, fitness);        
        % Crossover
        [offspring1, offspring2] = PerformCrossover(selected(1,:), selected(2,:));
        % Mutation
        offspring1 = Mutation(offspring1);
        offspring2 = Mutation(offspring2);
        % Calculate fitness
        %s1 = Fitness(offspring1, scenario);
        %s2 = Fitness(offspring2, scenario);
        [s1, timeOfVisitingS1] = FitnessMap(offspring1, m, scenario);
        [s2, timeOfVisitingS2] = FitnessMap(offspring2, m, scenario);
        % Population update
        pop(k,:) = offspring1;        
        fitness(k,:) = CalculateAreaTime(timeOfVisitingS1, s1);  
        timeOfVisitingPop(k, :) = timeOfVisitingS1;
        totalFlightTime(k,:) = s1;        
        UpdateBestFitnessMatrix(offspring1, fitness(k,:));                        
        k = k - 1;
        pop(k,:) = offspring2;        
        fitness(k,:) = CalculateAreaTime(timeOfVisitingS2, s2);  
        timeOfVisitingPop(k, :) = timeOfVisitingS2;
        totalFlightTime(k,:) = s2;
        UpdateBestFitnessMatrix(offspring2, fitness(k,:));                
        k = k - 1;        
 
        maxS = max(s1,s2);
        minS = min(s1,s2);
        if maxS > MAX_TIME             
             MAX_TIME = maxS;
             flagChangeTime = 1;                     
        end
        if minS < MIN_TIME
            MIN_TIME = minS;
            flagChangeTime = 1;                            
        end
        
    end        
    
    if flagChangeTime == 1 && USE_MAX_TIME == 1
        for q = 1: size(pop,1)                        
            [totalFlightTime(q), timeOfVisitingPop(q,:)] = FitnessMap(pop(q,:), m, scenario);              
        end
        fitness = CalculateAreaTimePopulation(pop, timeOfVisitingPop, totalFlightTime);            
        
        for q = 1: size(BEST_SOLUTIONS,1)                        
            [s1, timeOfVisitingS1] = FitnessMap([BEST_SOLUTIONS(q,:)], m, scenario);
            BEST_FITNESS(q) = CalculateAreaTimePopulation(BEST_SOLUTIONS(q,:), timeOfVisitingS1, s1);
                TIME_OF_FLIGHT(q) = s1;
        end
    end
       %pop = pop(1:popSize,:);
       

end

uavs = scenario.UAVs;

 best = pop(1,:);
%  if USE_MAX_TIME == 1
%      if (fitness(1) >= f*1.1)
%          best = array;    
%      end
%  else
%      if (fitness(1) <= f*0.9)
%          best = array;    
%      end
%  end
clone_uavs = ConvertArrayToUAVs(best, uavs);

for i = 1: clone_uavs.Count()
   uav = clone_uavs.Value(i);
   uav.FlightPath = FlightPath();
   goal = uav.GetNextGoalPosition(scenario);
   [newGoal] = GetNextGoalUAVRiskMap(uav,goal);
   newGoal = newGoal.AddLast(goal);
   uav.WaypointsToVisit = newGoal;
   if (uav.GroundForcesToVisit.Count()> 0)
       uav.UnderMission = 1;
   end
   clone_uavs = clone_uavs.ModifyValueInIndex(i,uav);
end
scenario.UAVs = clone_uavs;

if USE_MAX_TIME == 0
    [BEST_FITNESS,order] = sort(BEST_FITNESS);
else
    [BEST_FITNESS,order] = sort(BEST_FITNESS, 'descend');
end

BEST_SOLUTIONS = BEST_SOLUTIONS(order,:);

for i = 1: size(BEST_SOLUTIONS,1)
    if BEST_SOLUTIONS(i,1) > 0 
        [TIME_OF_FLIGHT(i), ~] = FitnessMap(BEST_SOLUTIONS(i,:), m, scenario);      
    end
end


    
end

function exist = ExistCromossome(solution)

global BEST_SOLUTIONS;
global BEST_FITNESS;
global INDEX_BEST;

for i = 1: INDEX_BEST
   
    if (sum(solution ~= BEST_SOLUTIONS(i,:)) == 0) 
        exist = 1;
        return;
    end
end
exist = 0;

end

function UpdateBestFitnessMatrix(solution, fitness)

global BEST_SOLUTIONS;
global BEST_FITNESS;
global INDEX_BEST;

if ExistCromossome(solution) == 0
    if INDEX_BEST == size(BEST_SOLUTIONS,1)
        [l,c] = find(BEST_FITNESS == min(BEST_FITNESS));
        l = l(1);
        c = c(1);
        if fitness > BEST_FITNESS(l,c)
            BEST_SOLUTIONS(l,:) = solution;
            BEST_FITNESS(l,c) = fitness;
        end
    else
        INDEX_BEST = INDEX_BEST + 1;
        BEST_SOLUTIONS(INDEX_BEST,:) = solution;
        BEST_FITNESS(INDEX_BEST) = fitness;
    end
end


end

function fx = CalculateAreaTimePopulation(pop, timeOfVisitingPop, totalFlightTime)

%global BEST_SOLUTIONS;
%global BEST_FITNESS;
%global INDEX_BEST;


n = size(timeOfVisitingPop,1);
fx = zeros(n,1);
for i = 1: n
    fx(i) = CalculateAreaTime(timeOfVisitingPop(i,:), totalFlightTime(i,:));    
    UpdateBestFitnessMatrix(pop(i,:), fx(i));
end

end

function exist = ExistElement(array, iMin, iMax, element)

exist = sum(array(iMin:iMax) == element) > 0;

end

function [child1, child2] = PerformCrossover(solution1, solution2)
n = length(solution1);
r1 = randi(n);
r2 = randi(n);
while (r1 == r2)
   r2 = randi(n);
end
child1 = OrderCrossover(solution1, solution2, r1, r2);
child2 = OrderCrossover(solution2, solution1, r1, r2);

end

function output = OrderCrossover(solution1, solution2, randomStart, randomEnd)

minIndex = min([randomStart, randomEnd]);
maxIndex = max([randomStart, randomEnd]);

i = 1;
output = zeros(1, length(solution1));
indexOutput = 1;
count = 0;

qtdMenos1Sub = sum(solution1(minIndex:maxIndex) == -1);
qtdTotalMenos1 = sum(solution2 == -1);

while (count < minIndex-1)

	tempValue = solution2(i);
    
    if tempValue > 0
        if (~ExistElement(solution1, minIndex, maxIndex, tempValue))
            count = count+1;
            output(indexOutput) = tempValue;
            indexOutput = indexOutput + 1;
        end
    elseif tempValue == -1
        qtdParcialMenos1 = sum(output == -1);
        if (qtdParcialMenos1 + qtdMenos1Sub) < qtdTotalMenos1        
            count = count+1;
            output(indexOutput) = tempValue;
            indexOutput = indexOutput + 1;
        end
    end        
	i = i + 1;

end

% insert values of vector 1
for i = minIndex:maxIndex
	output(indexOutput) = solution1(i);
	indexOutput = indexOutput + 1;
end

for i = 1:length(solution2)
	tempValue = solution2(i);
    if tempValue > 0
        if (~ExistElement(output, 1, length(solution2), tempValue))
            output(indexOutput) = tempValue;
            indexOutput = indexOutput + 1;
        end
    elseif tempValue == -1
        if sum(output == -1) < sum(solution2== -1)
            output(indexOutput) = tempValue;
            indexOutput = indexOutput + 1;
        end
    end
        
end
	

end

function [pop, scores, timeOfVisitingPop, totalFlightTime] = SortPop(pop, scores, timeOfVisitingPop, totalFlightTime)
global USE_MAX_TIME;

if USE_MAX_TIME == 0
    [scores,order] = sort(scores);
else
    [scores,order] = sort(scores, 'descend');
end
pop = pop(order,:);
timeOfVisitingPop = timeOfVisitingPop(order,:);
totalFlightTime = totalFlightTime(order,:);
end

function array = ConvertUavsToCromossome(uavs)

nUAVs = uavs.Count();
index = 1;
array = zeros(1, 500);
for i = 1: nUAVs
   uav = uavs.Value(i); 
   gfs = uav.GroundForcesToVisit;
   nGfs = gfs.Count();
   for j = 1: nGfs
      gfId =  gfs.Value(j);
      if (sum(array == gfId) == 0)
          array(index) = gfId;
          index = index + 1;
      end
   end
   if i ~= nUAVs
       array(index) = -1;
       index = index + 1;
   end
end

array = array(:, 1:index-1);

end

function [scoreFinal, timeOfVisiting] = FitnessMap(array, costMap, scenario)
    global EXPOSURE_TIME;
    
%     while (array(1) == -1)
%         array = array(2:end);
%     end
    timeOfVisiting = zeros(1,sum(array>0));
    indexTimeOfVisiting = 1;
    uavs = scenario.UAVs;
    clone_uavs = ConvertArrayToUAVs(array, uavs);    
    nUAVs = clone_uavs.Count();    
    scoreFinal = zeros(nUAVs, 1);
    param = Parameters();
    for i =1 : nUAVs
       score = 0;
       uav = clone_uavs.Value(i);
       if uav.Alive == 0 || uav.Exposed || EXPOSURE_TIME(uav.Id) >= param.ExposureTimeThreshold
           if uav.GroundForcesToVisit.Count()>0
               scoreFinal = inf;
               return;           
           else 
            continue;
           end
       end
           
       gfs = uav.GroundForcesToVisit;
       
       if gfs.Count() == 0           
        str = sprintf('UAV%dGCS', uav.Id);           
        v = costMap.values({str});
        score = score + v{1}; 
       else
          % Calculate cost from current uav position to gf
          str = sprintf('UAV%dGF%d', uav.Id, gfs.Value(1));           
          v = costMap.values({str});                    
          score = score + v{1};   
          % Update time of visiting
          timeOfVisiting(indexTimeOfVisiting) = score;
          indexTimeOfVisiting = indexTimeOfVisiting + 1;
          % Cost between GFs
          for j = 2: gfs.Count()
             str = sprintf('GF%dGF%d', gfs.Value(j-1), gfs.Value(j));           
             v = costMap.values({str});             
             score = score + v{1};         
             % Update time of visiting
             timeOfVisiting(indexTimeOfVisiting) = score;
             indexTimeOfVisiting = indexTimeOfVisiting + 1;
          end
          % Cost to return GCS
          str = sprintf('GF%dGCS', gfs.Value(gfs.Count()));           
          v = costMap.values({str});
          score = score + v{1}; 
          % Update time of visiting
          %timeOfVisiting(indexTimeOfVisiting) = score;
          %indexTimeOfVisiting = indexTimeOfVisiting + 1;
       end                            
       % Update time of visiting
       scoreFinal(i) = score;
    end     
    scoreFinal = max(scoreFinal);    
    %timeOfVisiting(end) = scoreFinal;
    timeOfVisiting = sort(timeOfVisiting);
    
end

function [pop, scores, timeOfVisiting] = CreatPopulation(array, nPop, scenario, costMatrix)

nArray = length(array);
pop = zeros(nPop, nArray);
scores = zeros(nPop, 1);
pop(1,:) = array;
%scores(1) = Fitness(array, scenario);
timeOfVisiting = zeros(1,sum(array>0));
[scores(1), timeOfVisiting(1,:)] = FitnessMap(array, costMatrix, scenario);
for i = 2: nPop
    pop(i,:) = array(randperm(nArray));
    %scores(i) = Fitness(pop(i,:), scenario);
    [scores(i), timeOfVisiting(i,:)] = FitnessMap(pop(i,:), costMatrix, scenario);  
end

end


function fx = CalculateFitnessArea(timeOfVisiting, maxTime)

    n = length(timeOfVisiting);
    
    % only one waypoint to visiting
    
    if n == 1
        fx = maxTime - timeOfVisiting(1);
    else
        fx = 0;
        for i = 1: n-1
            t1 = timeOfVisiting(i);
            t2 = timeOfVisiting(i+1);
            area = (t2-t1)*i;
            fx = fx + area;
        end
        area = (maxTime - timeOfVisiting(end))*n;
        fx = fx + area;
    end              
end


function fx = CalculateAreaTime(timeOfVisiting, timeOfFlight)
    global MAX_TIME;
    global MIN_TIME;

    n = length(timeOfVisiting);
    
    % only one waypoint to visiting
    k = 1.5;
    if n == 1
        fx = inf;
        return;
    else                
        area = timeOfVisiting(1);
        fx = area;
        for i = 1: n-1
            t1 = timeOfVisiting(i);
            t2 = timeOfVisiting(i+1);
            area = (t2-t1)*(i+1);
            fx = fx + area;
        end
        area = (timeOfFlight - timeOfVisiting(end))*n+1;                
        fx = fx + area;
        area = (MAX_TIME - timeOfFlight)*n+2;                
        fx = fx + area;
    end              
        
       fx = fx * (MIN_TIME / timeOfFlight);

    
end

function clone_uavs = ConvertArrayToUAVs(array, uavs)

clone_uavs = repmat(uavs,1);
nUAVs = clone_uavs.Count();
nArray = length(array);
indexUAV = 1;
for i = 1: nUAVs
    uav = clone_uavs.Value(i);
    uav.GroundForcesToVisit = List();
    clone_uavs = clone_uavs.ModifyValueInIndex(i,uav);
end

for i = 1: nArray
   if (array(i) ~= -1)       
           uav = clone_uavs.Value(indexUAV);
           uav.GroundForcesToVisit = uav.GroundForcesToVisit.AddLast(array(i));
           clone_uavs = clone_uavs.ModifyValueInIndex(indexUAV,uav);       
   else
       indexUAV = indexUAV + 1;
   end
    
end

end

function selected = TournamentSelection(pop, scores)
global USE_MAX_TIME;
nPop = size(pop,1);
k = 0.75;
n = 8;

selected = zeros(n, size(pop,2));
for i = 1: n
    r1 = randi(nPop);
    c1 = scores(r1);    
    r2 = randi(nPop);
    c2 = scores(2);
    
    r = rand();
    if USE_MAX_TIME == 1 
        if r < k
            if (c1 < c2)
                selected(i,:) = pop(r1,:);
            else
                selected(i,:) = pop(r2,:);
            end
        else
            if (c1 < c2)
                selected(i,:) = pop(r2,:);
            else
                selected(i,:) = pop(r1,:);
            end
        end
    else
        if r < k
            if (c1 > c2)
                selected(i,:) = pop(r1,:);
            else
                selected(i,:) = pop(r2,:);
            end
        else
            if (c1 > c2)
                selected(i,:) = pop(r2,:);
            else
                selected(i,:) = pop(r1,:);
            end
        end
    end
end

end

function result = Mutation(array)

result = repmat(array,1);

if rand() < 0.10
    n = length(array);
    r1 = randi(n);
    r2 = randi(n);
    while (r1 == r2)
       r2 = randi(n);
    end

    minIndex = min([r1, r2]);
    maxIndex = max([r1, r2]);

    
    result(minIndex:maxIndex) = array(maxIndex:-1:minIndex);
end

end