classdef Position3D
    properties
        X;
        Y;
        Z;
    end
    methods
        
        % Constructor
        function obj = Position3D(x,y,z)            
            if (nargin < 0 || nargin > 3)
                error('prog:input', 'Constructor of class Position - Invalid number of inputs. Received %d.', nargin);
            elseif nargin == 0
                obj.X = -1;
                obj.Y = -1;
                obj.Z = -1;
            elseif nargin == 1
                obj.X = x.X;
                obj.Y = x.Y;
                obj.Z = x.Z;
            elseif nargin == 2
                obj.X = x;
                obj.Y = y;
                obj.Z = -1;
            elseif nargin == 3
                obj.X = x;
                obj.Y = y;
                obj.Z = z;
            end            
        end        
        
        % Overload of operator equal         
        function res = eq(input1, input2)
            res = 0;
            if (input1.X == input2.X && ...
                input1.Y == input2.Z && ...
                input1.Y == input2.Z)
                res = 1;
            end            
        end
        
        function output = GetAsArray2D(obj)
           output = [obj.X,  obj.Y];
        end
        
        function output = GetAsArray3D(obj)
           output = [obj.X,  obj.Y, obj.Z];
        end
        
        function dist = CalculateDistancePos(obj, pos)
           dist = sqrt((obj.X - pos.X)^2 + ...
                       (obj.Y - pos.Y)^2 + ...
                       (obj.Z - pos.Z)^2);
        end
    end
end