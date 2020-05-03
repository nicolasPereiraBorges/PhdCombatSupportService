classdef Threat
    properties
        Id;        
        DetectionRange;
        WeaponRange;        
        Position;
    end
    methods
        
        % Constructor
        function obj = Threat(id, detectionRange, weaponRange, position)
            if (nargin < 0 || nargin > 4)
                error('prog:input', 'Constructor of class Threat - Invalid number of inputs. Received %d', nargin);
            elseif nargin == 0
                obj.Id = -1;                
                obj.DetectionRange = -1;
                obj.WeaponRange = -1;
                obj.Position = Position3D();
            elseif nargin == 1
                obj.Id = id.Id;                
                obj.DetectionRange = id.DetectionRange;                
                obj.WeaponRange = id.WeaponRange;                
                obj.Position = id.Position;
            elseif nargin == 3
                obj.Id = id;                
                obj.DetectionRange = detectionRange;                
                obj.WeaponRange = weaponRange;                
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