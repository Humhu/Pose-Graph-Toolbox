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
    
    methods(Static)
        
        % Groups and averages measurements into a compact set of unique
        % relations
        function [m] = Compact(many)
            
            grouped = MeasurementRelativePose.Group(many);
            m = cell(size(grouped));
            for i = 1:numel(grouped)
                m{i} = MeasurementRelativePose.Average(grouped{i});
            end
            
        end
        
        % Averages multiple measurements of the same relation into a single
        % equivalent measurement. Expects input in array form.
        function [m] = Average(many)
            
            if any(~many(1).SameRelation(many))
                error('Cannot compact measurements of different relations');
            end
            
            % Can't compute weighted-sum of angles directly - Need to go to
            % intermmediate overparameterized format. Here we use [cos;
            % sin] unit vectors instead.
            means = [many.double()];
            pos_means = means(1:2,:);
            angles = means(3,:);
            angle_means = [cos(angles); sin(angles)];            
            
            weighted_pos_mean = zeros(2,1);
            sum_pos_info = zeros(2);
            weighted_angle_mean = zeros(2,1);
            sum_angle_info = zeros(2);
            
            for i = 1:numel(many)
                cov = many(i).covariance;
                pos_cov = cov(1:2, 1:2);
                angle_cov = cov(3,3)*eye(2);                
                pos_info = inv(pos_cov);
                angle_info = inv(angle_cov);
                
                sum_pos_info = sum_pos_info + pos_info;
                sum_angle_info = sum_angle_info + angle_info;
                
                weighted_pos_mean = weighted_pos_mean + pos_info*pos_means(:,i);
                weighted_angle_mean = weighted_angle_mean + angle_info*angle_means(:,i);
            end
            pos_mean = sum_pos_info\weighted_pos_mean;
            angle_mean = sum_angle_info\weighted_angle_mean;
            
            pos_cov = inv(sum_pos_info);
            angle_cov = inv(sum_angle_info);                        
            cov = [pos_cov, [0;0];
                   0, 0, angle_cov(1)];            
            m = many(1);
            m.displacement = pos_mean;
            m.rotation = wrapToPi(atan2(angle_mean(2), angle_mean(1)));
            m.covariance = cov;
        end
        
        % Groups measurements into a cells of arrays for unique relations
        function [grouped] = Group(many)
            
            unq = [];
            grouped = {};
            for i = 1:numel(many)
                z = many(i);
                match = z.SameRelation(unq);
                if ~any(match)
                   unq = [unq, z];
                   grouped{end + 1} = z;
                else
                   grouped{match} = [grouped{match}, z]; 
                end
                
            end
            
        end
        
    end
    
end