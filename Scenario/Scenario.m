classdef Scenario
    %SCENARIO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        GCS;
        GroundForces;
        UAVs;
        Threats;                
    end
    
    methods
        function obj = Scenario(nUAVs, nGFs, nThreats)
            if nargin ~= 3
                error('prog:input', 'Constructor of class Scenario - Invalid number of inputs. Received %d.', nargin);
            else
                obj.GCS = CreateGCS();
                obj.UAVs = CreateUavs(obj.GCS, nUAVs);
                obj.GroundForces = CreateGroundForces(obj.GCS, nGFs);
                obj.Threats = CreateThreats(obj.GCS, nThreats, obj.GroundForces);
            end            
        end        
    end
end

