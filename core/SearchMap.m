% Maps from one set to another using search
classdef SearchMap < handle
    
    properties
        
        input_set;
        output_set;        
        
    end
    
    methods
    
        function [obj] = SearchMap(in, out)
           
            obj.input_set = in;
            obj.output_set = out;
            
        end
        
        function [out] = Forward(obj, in)
            
            [~, ~, i] = intersect(in, obj.input_set);
            out = obj.output_set(i)';
            
        end
        
        
        function [in] = Backward(obj, out)
           
            [~, ~, i] = intersect(out, obj.output_set);
            in = obj.input_set(i)';
            
        end
        
    end
    
end
      