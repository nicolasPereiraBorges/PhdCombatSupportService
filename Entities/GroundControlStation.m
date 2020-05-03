classdef GroundControlStation
    properties
        Id;        
        Position;        
    end
    methods
        
        % Constructor
        function obj = GroundControlStation(id, position)
            if nargin < 0 || nargin > 2
                error('prog:input', 'Constructor of class GroundControlStation - Invalid number of inputs. Received %d', nargin);
            elseif nargin == 0
                obj.Id = -1;
                obj.Position =  Position3D();
            elseif nargin == 1
                obj.Id = position.Id;                
                obj.Position = position.Position;            
            elseif nargin == 2
                obj.Id = id;                
                obj.Position = position;            
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