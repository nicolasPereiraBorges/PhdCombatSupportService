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
        Exposed;
        PathHistory;
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
                obj.Exposed = 0;
                obj.PathHistory = List();
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
                obj.Exposed = battery.Exposed;
                obj.PathHistory = battery.PathHistory;
                
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
                obj.Exposed = 0;
                obj.PathHistory = List();
            end
        end                
        
        function obj = UpdatePathHistory(obj)
           
            obj.PathHistory = obj.PathHistory.AddLast(obj.Position);
            if obj.PathHistory.Count() > 5
                obj.PathHistory = obj.PathHistory.RemoveValueInIndex(1);
            end
        end
        
        function pos = GetPathHistoryAsArray(obj)
           n = obj.PathHistory.Count();
           if n == 0
               pos = [];
           else
               pos = zeros(n, 2);               
               for i = 1: n
                   pos(i,1:2) = obj.PathHistory.Value(i).GetAsArray2D();
               end
           end          
        end
        
        function goal = GetNextGoalPosition(obj, gcs)            
            if obj.GroundForcesToVisit.Count() > 0
                goal_index = obj.GroundForcesToVisit.Value(1);
                goal = gcs.GroundForces.Value(goal_index).Position;
            else
                goal = obj.Gcs.Position;
            end
        end
        
        function goal = GetNextGroundForce(obj)        
            if obj.GroundForcesToVisit.Count() > 0
                goal = obj.GroundForcesToVisit.Value(1);
            else
                goal = [];
            end
        end
        
        
        function goal = GetPathToPlot(obj)        
            if obj.FlightPath.Count() > 0
                goal = obj.GroundForcesToVisit.Value(1);
            else
                goal = [];
            end
        end
        
        function gf = GroundForcesToVisitAsString(obj)        
            gf = 'GF=(';
            if obj.GroundForcesToVisit.Count() > 0
                for i = 1: obj.GroundForcesToVisit.Count()
                   gf =  [gf, num2str(obj.GroundForcesToVisit.Value(i)), ','];
                end
                gf = gf(1:end-1);              
            end
            gf = [gf,')'];                                    
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