% Relative pose measurement class
% Noise is 0 centered and Gaussian
classdef MeasurementRelativePose < Measurement
    
    properties
        
        displacement;
        rotation = Orientation1D;
        covariance;
        observer_id;
        target_id;
        observer_time;
        target_time;
        
    end
    
    methods
        
        function obj = MeasurementRelativePose(obs_p, tar_p, cov)          
            if nargin == 0
                return
            end
            
            rel = tar_p - obs_p;            
            
            ori = rel.orientation;
            noise = mvnrnd(zeros(3,1), cov)';
            t = double(obs_p.orientation);
            R = [cos(t), sin(t);
                -sin(t), cos(t)];
            
            obj.displacement = R*rel.position + noise(1:2,1);
            obj.rotation = ori + noise(3);
            obj.covariance = cov;
            
        end
        
        function D = double(obj)
            
            D = [obj.displacement; double(obj.rotation)];
            
        end
        
    end
    
end