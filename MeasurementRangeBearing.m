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
        
        % TODO: Figure out what to do with non-invertible measurements
%         function [estPose] = ToPose(obj, basePose)
%            
%             a = double(basePose.orientation + obj.bearing);
%             dx = m.range*cos(a);
%             dy = m.range*sin(a);                                    
%             estPose = 
%             
%         end
        
        function D = double(obj)
            
            D = [obj.range; double(obj.bearing); obj.covariance; ...
                obj.observer_id; obj.target_id];
            
        end
        
    end
    
end