% Represents an iterative localization problem
classdef EMIterate < handle
    
    properties
        
        true_plotter = Plotter2D;
        est_plotter = Plotter2D;
        true_world = World2D;
        est_world = World2D;
        
        step = 0;
        
        pose_beliefs = Pose2D;
        meas;
        meas_ids;
        step_magnitude = Inf;
        
    end
    
    methods
        
        function obj = EMIterate(true, est)
            
            if nargin == 0
                return
            end
            
            obj.true_world = true;
            obj.true_plotter = Plotter2D(obj.true_world.dims);
            obj.true_plotter.ReadSource(obj.true_world);
            obj.true_plotter.Label('True World');
            obj.true_plotter.PlotPoses();
            
            obj.est_world = est;
            obj.est_plotter = Plotter2D(obj.est_world.dims);
            obj.est_plotter.ReadSource(obj.est_world);
            obj.est_plotter.Label('Estimated World');
            obj.est_plotter.PlotPoses();
            
        end
        
        function [] = Update(obj)
            
            obj.pose_beliefs = obj.est_world.GetPoses();
            [obj.meas] = obj.true_world.GetMeasurements();
            
        end
        
        function [] = Solve(obj, tol, max_iters)
            
            while true
                
                if obj.step_magnitude < tol || obj.step > max_iters
                    break
                end
                
                obj.Iterate();
                
            end
            
        end
        
        function [] = Iterate(obj)
            
            % Calculate errors
            N = numel(obj.pose_beliefs);
            
            num_rels = numel(obj.meas);
            
            % Calculate maximum likelihood errors
            sum_info = zeros(3,3,N); % Denominator - sum of covariances
            sum_data = zeros(3,1,N); % Numerator - sum of weighted data
            for k = 1:num_rels
                
                z = obj.meas{k};
                r = z.displacement;
                a = double(z.rotation);
                i = z.observer_id;
                j = z.target_id;
                c = z.covariance;
                
                pi = obj.pose_beliefs(i);
                pix = pi.position;
                pit = pi.orientation;
                
                t = double(pit);
                R = [cos(t), -sin(t);
                    sin(t), cos(t)];
                pj_est = [pix + R*r;
                    double(pit + a)];
                
                w = c^-1;
                sum_data(:,:,j) = sum_data(:,:,j) + w*pj_est;
                sum_info(:,:,j) = sum_info(:,:,j) + w;
                
            end
            
            for k = 1:N
                
                data = sum_data(:,:,k);
                info = sum_info(:,:,k);
                
                p_new = (info^-1)*data;
                p_new = reshape(p_new, 1, 1, 3);
                obj.pose_beliefs(k) = Pose2D(p_new(:,:,1:2), p_new(3));
                
            end
            
            obj.step_magnitude = norm(dx);
            
            d = shiftdim(reshape(dx, 1, 3, N), 2);
            dPoses = Pose2D(d(:,:,1:2), d(:,:,3));
            obj.pose_beliefs = obj.pose_beliefs + dPoses;
            
            obj.step = obj.step + 1;
            
            obj.Visualize();
            
        end
        
        function [] = Visualize(obj)
            
            obj.true_plotter.ReadSource(obj.true_world);
            obj.true_plotter.PlotPoses();
            obj.true_plotter.PlotMeasurements(obj.meas, obj.meas_ids);
            
            obj.est_plotter.ReadSource(obj.pose_beliefs);
            obj.est_plotter.PlotPoses();
            obj.est_plotter.PlotMeasurements(obj.meas, obj.meas_ids);
            
        end
        
    end
    
end