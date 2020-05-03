function PlotScenario(scenario, iteration)
%Aeronautics Institute of Technology
%Author: Nicolas Pereira Borges - nicolas@ita.br
%Date: 07/11/2016

    % Get all parameters
    param = Parameters;       
    
    uavs = scenario.UAVs;
    threats = scenario.Threats;
    gfs = scenario.GroundForces;
    gcs = scenario.GCS;
        
        
        %Clear screen and plot base points
        cla reset
        hold all;    
        xlim([-200, param.ScenarioWidth + 200]);
        ylim([-200, param.ScenarioHeight + 200]);
        % Plot threats
        PlotPosition(threats)
        % Plot Edges
        PlotPosition(gfs); 
        % Plot UAVs
        PlotPosition(uavs);               
        %plot(gcs.Position.X, gcs.Position.Y, '.g', 'markersize', 30); 
        %text(gcs.Position.X - 100, gcs.Position.Y + 150, "GCS"); 
        title("Simulation time = " + num2str(iteration));        
        
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
            plot(x, y, '.k', 'Markersize', 10);                    
            text(x-100, y+150, "UAV-"+num2str(object.Id) );                
            end
        elseif isa(object, 'GroundForce')
            plot(x, y, '.b', 'Markersize', 20);                                    
            text(x-100, y+150, "GF-"+num2str(object.Id) );                
        else % Threat            
            plot(x, y, '.r', 'Markersize', 20);                                
            [xx,yy] = GeneratePointsCircle(object.DetectionRange, x, y);
            plot(xx,yy, 'r', 'linewidth', 2);
            text(x-100, y+150, "Threat-"+num2str(object.Id) );                
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

