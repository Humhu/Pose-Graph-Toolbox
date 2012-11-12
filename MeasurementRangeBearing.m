% Container for measurements
classdef MeasurementRangeBearing < Measurement
    
    properties
        range;
        bearing;
        covariance;
        observer_id;
        target_id;
        observer_time;
        target_time;
        
    end
    
    methods
        
        function obj = MeasurementRangeBearing(obs_p, tar_p, cov)
            if nargin == 0
                return
            end
            
            rel = tar_p - obs_p;
            r = norm(rel.position);
            noise = mvnrnd(zeros(2,1), cov)';
            r = r + noise(1);
            if r < 0
                r = 0;
            end
            b = atan2(rel.position(2), rel.position(1)) - obs_p.orientation;
            
            obj.range = r;
            obj.bearing = b;
            
            
        end
        
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