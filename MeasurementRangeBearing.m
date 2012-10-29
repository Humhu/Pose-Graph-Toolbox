% Container for measurements
classdef MeasurementRangeBearing
    
    properties
        range;
        bearing = Orientation1D;
        covariance;        
    end
    
    methods
        
        function obj = MeasurementRangeBearing(range, bearing, covariance)
            if nargin == 0
                return
            end
            
            obj.range = range;
            obj.bearing = bearing;
            obj.covariance = covariance;
            
        end                
        
        function D = double(obj)
                       
            D = zeros(size(obj,1), size(obj,2),2);
            D(:,:,1) = reshape([obj.range], size(obj));
            D(:,:,2) = double(reshape([obj.bearing], size(obj)));
            
        end
        
    end
    
end