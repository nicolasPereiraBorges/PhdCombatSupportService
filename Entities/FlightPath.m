classdef FlightPath
    properties            
        Positions;                
        Length;
    end
    methods
        
        % Constructor
        function obj = FlightPath(positions, length)
            if nargin < 0 || nargin > 2
                error('prog:input', 'Constructor of class FlightPath - Invalid number of inputs. Received %d', nargin);
            elseif nargin == 0
                obj.Positions = List();
                obj.Length =  - 1;
            elseif nargin == 1
                obj.Positions = length.Position;            
                obj.Length = length.Length;             
            elseif nargin == 2
                obj.Positions = positions;                
                obj.Length = length;            
            end
        end                                
        
        function value = First(obj)
        
            value = obj.Positions.First();
        end
        
        function obj = UpdatePosGivenArray(obj, array)
           obj.Positions = List();
           obj = AddPosGivenArray(obj, array);
        end
        
        function obj = AddPosGivenArray(obj, array)
           n = size(array);
           for i = 1: n(1)
              x = array(i,1); 
              y = array(i,2); 
              p = Position3D(x,y);
              obj.Positions = obj.Positions.AddLast(p);
           end
        end
        
        function pos = GetPositionAsArray(obj)
           n = obj.Positions.Count();
           if n == 0
               pos = [];
           else
               pos = zeros(n, 2);               
               for i = 1: n
                   pos(i,1:2) = obj.Positions.Value(i).GetAsArray2D();
               end
           end          
        end
        
        function count = Count(obj)
           count = obj.Positions.Count(); 
        end
    end
end