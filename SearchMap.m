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
            
            out = obj.output_set(obj.input_set == in);
            
        end
        
        
        function [in] = Backward(obj, out)
           
            in = obj.input_set(obj.output_set == out);
            
        end
        
    end
    
end
      