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
        
        function [estPose] = ToPose(obj, basePose)                                                                                  
            
            pix = basePose(1:2);
            pit = basePose(3);
            
            t = pit;
            R = [cos(t), -sin(t);
                sin(t), cos(t)];
            pj_est = [pix + R*obj.displacement;
                wrapToPi(pit + obj.rotation)];
            %pj_est = reshape(pj_est, 1,1,3);
            %estPose = Pose2D(pj_est(1:2), pj_est(3));
            estPose = pj_est;
            
        end
        
        function D = double(obj)
            
            D = [obj.displacement; obj.rotation];
            
        end
        
    end
    
end