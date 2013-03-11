% Maps from one set to another using an array
classdef LinearMap < handle
    
    properties
        
        input_set;
        input_offset;
        output_set;
        output_offset;
        
    end
    
    methods
       
        function [obj] = LinearMap(in, out)
            
            obj.input_offset = min(in) - 1;
            obj.output_offset = min(out) - 1;
            
            in_shifted = in - obj.input_offset;
            out_shifted = out - obj.output_offset;
            
            in_range = max(in_shifted);
            out_range = max(out_shifted);
            
            obj.input_set = -1*ones(1, in_range);
            obj.output_set = -1*ones(1, out_range);
            
            obj.input_set(out_shifted) = in;
            obj.output_set(in_shifted) = out;
            
        end
        
        function [out] = Forward(obj, in)
            
            out = obj.output_set(in - obj.input_offset);
            
        end
        
        function [in] = Backward(obj, out)
            
            in = obj.input_set(out - obj.output_offset);
            
        end
        
    end
    
end