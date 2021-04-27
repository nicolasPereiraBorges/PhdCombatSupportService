function PlotScenario(scenario, iteration, figure, knowAreas)
%Aeronautics Institute of Technology
%Author: Nicolas Pereira Borges - nicolas@ita.br
%Date: 07/11/2016


    global BEST_SOLUTIONS;
    global BEST_FITNESS;
    global INDEX_BEST;
    global TIME_OF_FLIGHT;


    % Get all parameters
    param = Parameters;       
    
    uavs = scenario.UAVs;
    threats = scenario.Threats;
    gfs = scenario.GroundForces;
    gcs = scenario.GCS;
    cla(figure);
    %set(0, 'CurrentFigure', figure)
    %cla reset
    %subplot(1,4,1:3);
        factor = 1000;
        %Clear screen and plot base points
        
        hold all;    
        xlim([-factor, param.ScenarioWidth + factor]);        
        %xlim([-factor, 8100]);
        ylim([-factor, param.ScenarioHeight + factor]);
        
        %ylim([-3000, param.ScenarioHeight + 3000]);
        % Plot threats
        PlotPosition(threats)
        % Plot Edges
        PlotPosition(gfs); 
        % Plot UAVs
        PlotPosition(uavs);               
        plot(knowAreas(:,1), knowAreas(:,2), '.g', 'markersize', 5);
        plot(gcs.Position.X, gcs.Position.Y, '.m', 'markersize', 60); 
        text(gcs.Position.X - 100, gcs.Position.Y + 250, "GCS"); 
        title("Simulation time = " + num2str(iteration));          
        
        
        a = gca; % get the current axis;
        % set the width of the axis (the third value in Position) 
        % to be 60% of the Figure's width
        a.Position(3) = 0.6;
        % put the textbox at 75% of the width and 
        % 10% of the height of the figure
        
        delete(findall(gcf,'type','annotation'))
            
        saida = [BEST_SOLUTIONS, TIME_OF_FLIGHT, BEST_FITNESS];
        
        k = 0.6;
        annotation('textbox', [0.80, 0.7, 0.1, 0.1], 'String', "Best solutions on DVRP");    

        for i = 1: size(saida,1)
            strSaida = "";
            for j = 1: size(saida,2) - 2
                strSaida = strSaida + num2str(saida(i,j)) + " ";
            end                
            strSaida = strSaida + " --- T(x): " + num2str(saida(i,end-1));            
            strSaida = strSaida + " -- F(x): " + num2str(saida(i,end));
            annotation('textbox', [0.80, k, 0.1, 0.1], 'String', strSaida);    
            k = k - 0.03;
        end               
        
        
        
        
    
end

function PlotPosition(objects)
    %function PlotUAV(plotSimulation, uav)
    % Plot uav coordinates and radar
    % Inputs:    uav: matrix nx1 of struct uav
    %Aeronautics Institute of Technology
    %Author: Nicolas Pereira Borges - nicolas@ita.br
    %Date: 27/01/2017

    % Get number of UAVs
    nObjects = objects.Count();
    % For each uav
    for i = 1: nObjects
        % Get uav
        object = objects.Value(i);
        x = object.Position.X;
        y = object.Position.Y;
                
        % plot uav
        if isa(object, 'UAV')
            if object.Alive == 1
            %plot(uav.x, uav.y, '.k', 'Markersize', 22);        
            pos = object.FlightPath.GetPositionAsArray();
%             if ~isempty(pos)
%                 plot(pos(:,1), pos(:,2), 'k', 'linewidth', 0.5);
%             end
            plot(x, y, '.k', 'Markersize', 20);                    
            text(x-150, y+150, ['UAV-',num2str(object.Id)]);                
            text(x-200, y-150, object.GroundForcesToVisitAsString());                
            
            end
        elseif isa(object, 'GroundForce')
            if (object.Visited == 1)
                plot(x, y, '.b', 'Markersize', 40);                                    
            else
                plot(x, y, '.g', 'Markersize', 40);                                    
            end
            text(x-100, y+250, "GF-"+num2str(object.Id) );                
        else % Threat            
            color = 'y';
            if object.Detected ==1
                color = 'r';
            end
            plot(x, y, '.r', 'Markersize', 20);                                
            [xx,yy] = GeneratePointsCircle(object.DetectionRange, x, y);
            plot(xx,yy, color, 'linewidth', 2);
            text(x-150, y+220, "Threat-"+num2str(object.Id) );                
        end                                
                
    end
    
end
%     % Get vulnerable uavs
%     vulnerables = GetVulnerableUAVs(uavs);
%     if ~isempty(vulnerables)
%         plot(vulnerables(:,1), vulnerables(:,2) ,'.r', 'markersize', 12);            
%     end
function [xunit,yunit] = GeneratePointsCircle(radius, cx, cy)
%function [xunit,yunit] = GeneratePointsCircle(radius, cx, cy)
% Generate points that in circle (for further plot)
% Parameters: radius: degrees (float)
%             cx: degrees (float)
%             cy: degrees (float)
% Output: xunit = xRange
%         yunut = yRange
% Technological Institute of Aeronautics
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 20/09/2016

   th = 0:pi/50:2*pi;
   xunit = radius * cos(th) + cx;
   yunit = radius * sin(th) + cy;
   

end

