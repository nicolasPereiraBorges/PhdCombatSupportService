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
        function obj = Scenario()
            if nargin ~= 0
                error('prog:input', 'Constructor of class Scenario - Invalid number of inputs. Received %d.', nargin);
            else
                obj.GCS = CreateGCS();
                obj.UAVs = CreateUavs(obj.GCS);
                obj.GroundForces = CreateGroundForces(obj.GCS);
                obj.Threats = CreateThreats(obj.GCS, obj.GroundForces);
            end            
        end        
    end
end

