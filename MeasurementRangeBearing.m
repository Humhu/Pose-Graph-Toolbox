% Container for measurements
classdef MeasurementRangeBearing < Measurement
    
    properties
        range;
        bearing = Orientation1D;
        covariance;
        observer_id;
        target_id;
    end
    
    methods
        
        function obj = MeasurementRangeBearing(rng, bea, cov, obs_id, tar_id)
            if nargin == 0
                return
            end
            
            obj.range = rng;
            obj.bearing = bea;
            obj.covariance = cov;
            obj.observer_id = obs_id;
            obj.target_id = tar_id;
        end                
        
        function D = double(obj)
                       
            D = [obj.range; double(obj.bearing); obj.covariance; ...
                obj.observer_id; obj.target_id];
            
        end
        
    end
    
end