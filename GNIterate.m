% Represents an iterative localization problem
classdef GNIterate < handle
    
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
        
        function obj = GNIterate(true, est)
            
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
            [obj.meas, obj.meas_ids] = obj.true_world.GetMeasurements();
            
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
            errors = zeros(N,N,2);
            
            ranges = [obj.meas.range];
            angles = [obj.meas.bearing];
            
            num_rels = size(obj.meas_ids, 2);
            
            for k = 1:num_rels
                
                i = obj.meas_ids(1,k);
                j = obj.meas_ids(2,k);
                
                pi = obj.pose_beliefs(i);
                pix = pi.position(1);
                piy = pi.position(2);
                pit = pi.orientation;
                pj = obj.pose_beliefs(j);
                pjx = pj.position(1);
                pjy = pj.position(2);
                
                r = ranges(k);
                a = angles(k);
                
                ex = pjx - pix - r*cos(double(a + pit));
                ey = pjy - piy - r*sin(double(a + pit));
                
                errors(i,j,1) = ex;
                errors(i,j,2) = ey;
                
            end
            
            % Generate Jacobian
            H = zeros(3*N);
            b = zeros(3*N,1);
            
            for k = 1:num_rels
                
                i = obj.meas_ids(1,k);
                j = obj.meas_ids(2,k);
                
                Jij = zeros(2, 3*N);
                
                pi = obj.pose_beliefs(i);
                ti = pi.orientation;
                
                r = ranges(k);
                a = angles(k);
                info = obj.meas(i).covariance^-1;
                
                dt = [  r*sin(double(ti + a));
                    -r*cos(double(ti + a))];
                Jij(:, 3*i - 2:3*i) = [-eye(2), dt];
                Jij(:, 3*j - 2:3*j) = [eye(2), zeros(2,1)];
                
                Hij = Jij'*info*Jij;
                H = H + Hij;
                
                e = reshape(errors(i,j,:), 2, 1);
                bij = e'*info*Jij;
                b = b + bij';
                
            end
            
            % Have to fix one pose or else system unsolvable
            H = [H; eye(3), zeros(3,3*(N-1))];
            b = [b; zeros(3,1)];
            
            dx = -H\b;
            
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