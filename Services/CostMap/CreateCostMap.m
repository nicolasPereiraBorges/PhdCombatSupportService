function cost = CreateCostMap(width, heigth)
%CREATECOSTMAP Summary of this function goes here
%   Detailed explanation goes here

  factor = 1000;
  %Clear screen and plot base points                
%   xlim([-factor, param.ScenarioWidth + factor]);        
%   xlim([-factor, 8100]);
%   ylim([-factor, param.ScenarioHeight + factor]);

  width = width + 3000;
  heigth = heigth + 3000;
x = zeros(heigth, width);
for i = 1: heigth
    x(i,:) = i;
end

y = zeros(heigth, width);
for i = 1: width
    y(:, i) = i;
end


cost = zeros(heigth, width x
?
    =-0987654321'
    ,0=
   =-0cost(end,end) = -100;
cost(end,end-1) = 100;
%cost = HeatMap(cost);

end