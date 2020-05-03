classdef List
    properties
        Elements;        
    end
    methods
        
        % Constructor
        function obj = List()
            if nargin ~= 0
                error('prog:input', 'Constructor of class List - Invalid number of inputs. Received %d', nargin);
            else
                obj.Elements = {};                                           
            end
        end                        
        % Insert
        function obj = AddLast(obj, value)
           obj.Elements = [obj.Elements, value];
        end        
        % Insert
        function obj = AddFirst(obj, value)
           obj.Elements = [value, obj.Elements];
        end        
        % Index
        function index = IndexOf(obj, value)
           index = -1;
            for i = 1: obj.Count()
               if (value == obj.Elements{i})
                   index = i;
                   return;
               end
           end
        end
        % Value
        function output = Value(obj, index)
           output = obj.Elements(index);
        end        
        % Remove
        function obj = RemoveValueInIndex(obj, index)
           if index == 1
               if (obj.Count() == 1)
                   obj.Elements = {};                   
               end
               obj.Elements = obj.Elements(2:end);
           else
               if index == obj.Count()
                   obj.Elements = obj.Elements(1:end-1);
               else               
                   obj.Elements = [obj.Elements(1:index-1), obj.Elements(index+1:end)];
               end
           end                   
        end
        % Remove
        function obj = RemoveValue(obj, value)
           index = obj.IndexOf(value);
           obj = obj.RemoveValueInIndex(index);
        end
        % Modify
        function obj = ModifyValueInIndex(obj, index, newValue)
            obj.Elements{index} = newValue;        
        end
        % Number of elements
        function res =  Count(obj)           
            res = length(obj.Elements);
        end
            
    end
end