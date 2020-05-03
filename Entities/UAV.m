classdef UAV
    properties
        Id;
        Battery;
        Speed;
        AngleRestriction;
        Position;
        FlightPath;
        Gcs;        
        GroundForcesToVisit;
        Threats;
        UnderMission;
        Alive;
    end
    methods
        
        % Constructor
        function obj = UAV(id, battery, speed, angleRestriction, position, flightPath, ...
                gcs, groundForcesToVisit, threats, underMission, alive)
            if nargin <0 || (nargin > 1 && nargin < 11) || nargin > 11
                error('prog:input', 'Constructor of class UAV - Invalid number of inputs. Received %d', nargin);
            elseif nargin == 0
                obj.Id = -1;
                obj.Battery = [];
                obj.Speed = -1;
                obj.AngleRestriction = -1;
                obj.Position = Position3D();                
                obj.FlightPath = FlightPath();
                obj.Gcs = GroundControlStation();
                obj.GroundForcesToVisit = List();
                obj.Threats = List();
                obj.UnderMission = 0;
                obj.Alive = 1;
            elseif nargin == 1
                obj.Id = battery.Id;
                obj.Battery = battery.Battery;
                obj.Speed = battery.Speed;
                obj.AngleRestriction = battery.AngleRestriction;
                obj.Position = battery.Position;                
                obj.FlightPath = battery.FlightPath;
                obj.Gcs = battery.GroundControlStation;
                obj.GroundForcesToVisit = battery.GroundForcesToVisit;
                obj.Threats = battery.Threats;
                obj.UnderMission = battery.UnderMission;
                obj.Alive = battery.Alive;
                
            elseif nargin == 5
                obj.Id = id;
                obj.Battery = battery;
                obj.Speed = speed;
                obj.AngleRestriction = angleRestriction;
                obj.Position = Position3D(position);
                obj.FlightPath = flightPath;
                obj.Gcs = gcs;
                obj.GroundForcesToVisit = groundForcesToVisit;
                obj.Threats = threats;
                obj.UnderMission = underMission;
                obj.Alive = alive;
            end
        end                
        
        % Overload of operator equal         
        function res = eq(input1, input2)
            res = 0;
            if (input1.Id == input2.Id)
                res = 1;
            end            
        end
    end
end