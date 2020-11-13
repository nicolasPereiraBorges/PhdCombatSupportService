function uavs = CheckFireShotRadar(uavs, uavIndex)
% function uavs = CheckFireShotRadar(uavs, uavIndex)
% Check if uav is will be destroyed by fire shoot radar
% Aeronautics Institute of Technology
% Author: Nicolas Pereira Borges - nicolas@ita.br
% Date: 14/11/2016

    % For each threat
    uav = uavs(uavIndex);
    for q = 1: size(uav.threats, 1)
            % Check if uav is in a lethal area
            if (uav.threats(q).weaponRange > 0)
                % Calculate probability of being destroyed
                dist = CalculateDistance(uav.x, uav.y, uav.threats(q).cx,  uav.threats(q).cy);                
                if dist <= uav.threats(q).weaponRange
                    prob = (uav.threats(q).weaponRange^2) / dist^2;
                    r = rand();
                    if r*100 < prob
                        % Destroy uav
                        uav.formationId = 1;
                        uav.destroyed = 1;
                        uav.path = [];
                        uavs(uavIndex) = uav;
                        % Exit
                        return;
                    end
                end
            end
    end
end