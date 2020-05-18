classdef GroundForce
    properties
        Id;        
        Position;
        Priority;
        Visited;
    end
    methods
        
        % Constructor
        function obj = GroundForce(id, position, priority, visited)
            if (nargin < 0 || nargin > 4 || nargin == 2 ||nargin == 3)
                error('prog:input', 'Constructor of class GroundForce - Invalid number of inputs. Received %d', nargin);
            elseif nargin == 0
                obj.Id = -1;                
                obj.Position = Position3D();
                obj.Priority = -1;
                obj.Priority = 0;
            elseif nargin == 1
                obj.Id = id.Id;                
                obj.Position = id.Position;                
                obj.Priority = id.Priority;                
                obj.Visited = id.Visited;                
            elseif nargin == 4
                obj.Id = id;                
                obj.Position = position;                
                obj.Priority = priority;                
                obj.Visited = visited;
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