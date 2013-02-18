% Relative pose measurement class
% Noise is 0 centered and Gaussian
% TODO: Write reverseMeasurement method to generate opposite measurements
classdef MeasurementRelativePose < Measurement
    
    properties
        
        displacement;
        rotation;
        covariance;
        observer_id;
        target_id;
        observer_time;
        target_time;
        
    end
    
    methods
        
        %TODO: move noise addition to sensor-side?
        function obj = MeasurementRelativePose(obs_p, tar_p, cov)
            if nargin == 0
                return
            end
            
            % Find relative position
            rel = tar_p - obs_p;
            pos = rel(1:2);
            ori = wrapToPi(rel(3));
            
            % Add noise and rotate into observer frame
            noise = mvnrnd(zeros(3,1), cov)';
            t = obs_p(3);
            R = [cos(t), sin(t);
                -sin(t), cos(t)];
            
            obj.displacement = R*pos + noise(1:2,1);
            obj.rotation = ori + noise(3);
            obj.covariance = cov;
            
        end
        
        % Reverses measurement
        function [invM] = ToInverse(obj)
            
            %pz = Pose2D(zeros(1,1,2), 0);
            pz = zeros(3,1);
            pe = obj.ToPose(pz);
            invM = MeasurementRelativePose(pe, pz, zeros(3));
            invM.covariance = obj.covariance;
            invM.observer_id = obj.target_id;
            invM.target_id = obj.observer_id;
            invM.observer_time = obj.target_time;
            invM.target_time = obj.observer_time;
            
        end
        
        % Computes the target position corresponding to this observation
        % taken from basePose
        function [tarPose] = ToPose(obj, basePose)
            
            pix = basePose(1:2);
            pit = basePose(3);
            
            t = pit;
            R = [cos(t), -sin(t);
                sin(t), cos(t)];
            pj_est = [pix + R*obj.displacement;
                wrapToPi(pit + obj.rotation)];
            tarPose = pj_est;
            
        end
        
        function D = double(obj)
            
            D = [obj.displacement; obj.rotation];
            
        end
        
        function [m] = Compose(obj, z)
            
            if(obj.target_id ~= z.observer_id)
                error('Mismatching IDs in composition: a.tar = %d, b.obs = %d', ...
                    obj.target_id, z.observer_id);
            end
            if(obj.target_time ~= z.observer_time)
                error('Mismatching times in composition: a.time = %d, b.time = %d', ...
                    obj.target_time, z.observer_time);
            end
            
            obs_p = zeros(3,1);
            mid_p = obj.ToPose(obs_p);
            tar_p = z.ToPose(mid_p);
            a = mid_p(3);
            R = [cos(a), -sin(a), 0;
                 sin(a), cos(a), 0
                 0,      0,      1];
            cov = obj.covariance + R*z.covariance*R';
            
            m = MeasurementRelativePose(obs_p, tar_p, zeros(3));
            m.covariance = cov;
            m.observer_id = obj.observer_id;
            m.target_id = z.target_id;
            m.observer_time = obj.observer_time;
            m.target_time = z.target_time;
             
        end
        
    end
    
end