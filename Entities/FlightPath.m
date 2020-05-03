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
    end
end