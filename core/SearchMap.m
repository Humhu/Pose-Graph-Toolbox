% Maps from one set to another using search
classdef SearchMap < handle
    
    properties
        
        input_set;
        output_set;        
        
    end
    
    methods
    
        function [obj] = SearchMap(in, out)
           
            obj.input_set = reshape(in, 1, numel(in));
            obj.output_set = reshape(out, 1, numel(out));
            
        end
        
        function [out] = Forward(obj, in)
            
            %[~, ~, i] = intersect(in, obj.input_set);
            in = reshape(in, 1, numel(in));
            [i, ~] = find(bsxfun(@eq, obj.input_set', in));
            out = obj.output_set(i)';
            
        end
        
        
        function [in] = Backward(obj, out)
           
            [~, ~, i] = intersect(out, obj.output_set);
            in = obj.input_set(i)';
            
        end
        
    end
    
end
      