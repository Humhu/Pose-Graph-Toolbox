% Container for measurements
classdef MeasurementRelativePose < Measurement
    
    properties
        displacement;
        rotation = Orientation1D;
        covariance;
        observer_id;
        target_id;
    end
    
    methods
        
        function obj = MeasurementRelativePose(dis, rot, cov, obs_id, tar_id)
            if nargin == 0
                return
            end
            
            if(isa(rot, 'double'))
                rot = Orientation1D(rot);
            end
            
            obj.displacement = dis;
            obj.rotation = rot;
            obj.covariance = cov;
            obj.observer_id = obs_id;
            obj.target_id = tar_id;
            
        end
        
        function D = double(obj)
            
            D = [obj.displacement; double(obj.rotation)];
            
        end
        
    end
    
end