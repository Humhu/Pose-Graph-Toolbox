% Container for measurements
classdef Measurement
    
    properties
        mean;
        variance;        
    end
    
    methods
        
        function obj = Measurement(mean, variance)
            if nargin == 0
                return
            end
            
            obj.mean = mean;
            obj.variance = variance;
            
        end                
        
    end
    
end