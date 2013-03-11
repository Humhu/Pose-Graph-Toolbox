% Optimizes a graph using Gauss-Newton iteration
% Replaces older GNIterate
classdef GNSolver < handle
    
    properties
        
        tolerance;
        max_iterations;
        
    end
    
    properties(Access = private)
        
        idMap;
        tMap;
        
        iterations;
        residual;
        
    end
    
    methods
        
        function [obj] = GNSolver(tol, max_iters)
            
            if nargin == 0
                return
            end
            
            obj.tolerance = tol;
            obj.max_iterations = max_iters;
            
            obj.iterations = 0;
            
        end
        
        function [new] = Copy(obj)
            
            new = GNSolver(obj.tolerance, obj.max_iterations);
            
        end
        
        % Anchor is [anchor_id, anchor_time]
        function [solution, cov] = Solve(obj, sequence, anchor)
            
            % Need maps for IDs and time to indices
            [obj.idMap, obj.tMap] = sequence.BuildMaps();
            
            if nargin == 2
                anchor = [sequence(1).ids(1), sequence(1).time];
            end
            
            rem = Inf;
            solution = sequence;
            % Measurements don't change per iteration
            measurements = FlattenCell({solution.measurements});
            
            while(norm(rem) > obj.tolerance)
                [solution, rem, cov] = obj.Iterate(solution, measurements, anchor);
                obj.iterations = obj.iterations + 1;
                %fprintf(['Iteration: ', num2str(obj.iterations), ...
                %    '\tDelta max: ', num2str(norm(rem)), '\n']);
            end
            
        end
        
    end
    
    methods(Access = private)
        
        function [solution, delta, covs] = Iterate(obj, sequence, measurements, anchor)
            
            T = numel(sequence);
            N = sequence.GetDimension();
            
            % System Jacobian matrix
            H = zeros(3*N*T, 3*N*T);
            b = zeros(3*N*T, 1);
            
            for i = 1:numel(measurements)
                
                m = measurements{i};
                
                % Map into sequence indices
                obs_t = obj.tMap.Forward(m.observer_time);
                tar_t = obj.tMap.Forward(m.target_time);
                obs_id = obj.idMap.Forward(m.observer_id);
                tar_id = obj.idMap.Forward(m.target_id);
                if isempty(obs_t) || isempty(tar_t)
                    continue;
                end
                %used = [used, {m}];
                
                % Retrieve relevant poses
                obs_p = sequence(obs_t).poses(:, obs_id);
                tar_p = sequence(tar_t).poses(:, tar_id);
                
                % Precompute some terms for later
                ca1 = cos(obs_p(3));
                sa1 = sin(obs_p(3));
                R = [ca1, sa1;
                    -sa1, ca1];
                dR = [-sa1, ca1;
                    -ca1, -sa1];
                dX = tar_p(1:2) - obs_p(1:2);
                dA = tar_p(3) - obs_p(3);
                z = [m.displacement; m.rotation];
                %info = inv(m.covariance);
                
                % Calculate measurement error
                e = [R*dX; dA] - z;
                e(3) = wrapToPi(e(3));
                
                % Calculate measurement function Jacobian
                J_obs = [-R,      dR*dX;
                    zeros(1,2),   -1];
                J_tar = [R,          zeros(2,1);
                    zeros(1,2), 1];
                J = [J_obs, J_tar];
                %Hij = J'*info*J;
                Hij = J'*(m.covariance\J);
                %bij = e'*info*J;
                bij = e'*(m.covariance\J);
                
                % Add matrix into appropriate blocks
                obs_istart = 3*N*(obs_t - 1) + 3*(obs_id-1) + 1;
                tar_istart = 3*N*(tar_t - 1) + 3*(tar_id-1) + 1;
                inds = [obs_istart:obs_istart + 2, tar_istart:tar_istart + 2];
                H(inds,inds) = H(inds, inds) + Hij;
                b(inds) = b(inds) + bij';
                
            end
            
            % Anchor robot 1 at t0 with fake reading
            % TODO: Better way of doing this?
            J0 = eye(3);
            info = 1E6*eye(3);
            ai_ind = obj.idMap.Forward(anchor(1));
            at_ind = obj.tMap.Forward(anchor(2));
            i = 3*N*(at_ind - 1) + 3*(ai_ind - 1) + 1;
            H(i:i+2,i:i+2) = H(i:i+2,i:i+2) + J0'*info*J0;
            
            % Clean up states with no information and solve for increment
            for i = 1:size(H,1)
                if H(i,i) == 0
                    H(i,i) = 1;
                end
            end
            delta = H\-b;
            
            % Apply state increment to state
            solution = sequence;
            for t = 1:T
                for n = 1:N
                    pos = solution(t).poses(:,n);
                    inds = 3*N*(t - 1) + 3*(n - 1)  + 1;
                    pos = pos + delta(inds:inds + 2);
                    pos(3) = wrapToPi(pos(3));
                    solution(t).poses(:,n) = pos;
                end
            end
            
            % Store diagonal covariances
            covs = inv(H);
            
        end
        
    end
    
end

