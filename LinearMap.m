% Maps from one set to another using an array
classdef LinearMap < handle
    
    properties
        
        input_set;
        input_offset;
        input_valid;
        output_set;
        output_offset;
        output_valid;
        
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
            obj.input_set(out_shifted) = in;
            obj.output_set = -1*ones(1, out_range);                        
            obj.output_set(in_shifted) = out;
            
            obj.input_valid = false(1, in_range);
            obj.input_valid(out_shifted) = true;
            obj.output_valid = false(1, out_range);
            obj.output_valid(in_shifted) = true;
            
        end
        
        function [out] = Forward(obj, in)
            
            shifted = in - obj.input_offset;
            if any(shifted > numel(obj.output_set)) || ...
               any(shifted <= 0) || any(~obj.output_valid(shifted))
                out = [];
                return;
            end

            out = obj.output_set(shifted);
            
        end
        
        function [in] = Backward(obj, out)
            
            shifted = out - obj.output_offset;
            if any(shifted > numel(obj.input_set)) || ...
               any(shifted <= 0) || any(~obj.input_valid(shifted))
                in = [];
                return;
            end
            in = obj.input_set(shifted);
            
        end
        
    end
    
end