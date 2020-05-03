classdef Battery
    properties
        Id;                
    end
    methods
        
        % Constructor
        function obj = Battery(id)
            if nargin ~= 0 
                error('prog:input', 'Constructor of class Battery - Invalid number of inputs. Received %d', nargin);
            elseif nargin == 0
                obj.Id = -1;                
            elseif nargin == 1
                obj.Id = id.Id;                     
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