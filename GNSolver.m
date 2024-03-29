% Optimizes a graph using Gauss-Newton iteration
% Replaces older GNIterate
classdef GNSolver < handle
    
    properties
        
        tolerance;
        max_iterations;
        
        iterations;
        residual;
        covariance;
        
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
        
        function [solution, cov] = Solve(obj, sequence)
            
            rem = Inf;
            solution = sequence;
            while(norm(rem) > obj.tolerance)
                [solution, rem, cov] = obj.Iterate(solution);
                obj.iterations = obj.iterations + 1;
                fprintf(['Iteration: ', num2str(obj.iterations), ...
                    '\tDelta max: ', num2str(norm(rem)), '\n']);
            end
            
        end
        
        % TODO: 
        function [solution, delta, cov] = Iterate(obj, sequence)
            
            T = numel(sequence);
            N = size(sequence(1).poses,2);
            
            
            % System Jacobian matrix
            H = zeros(3*N*T, 3*N*T);
            b = zeros(3*N*T, 1);
            
            for t = 1:T                
                meas_t = sequence(t).measurements;
                M = numel(meas_t);
                for i = 1:M
                    m = meas_t{i};
                    obs_t = m.observer_time;
                    tar_t = m.target_time;
                    obs_id = m.observer_id;
                    tar_id = m.target_id;
                    
                    obs_p = sequence(obs_t).poses(:, obs_id);
                    tar_p = sequence(tar_t).poses(:, tar_id);
                    
                    ca1 = cos(obs_p(3));
                    sa1 = sin(obs_p(3));
                    
                    R = [ca1, sa1;
                        -sa1, ca1];
                    dR = [-sa1, ca1;
                        -ca1, -sa1];
                    dX = tar_p(1:2) - obs_p(1:2);
                    dA = tar_p(3) - obs_p(3);
                    z = [m.displacement; m.rotation];
                    info = inv(m.covariance);
                    
                    % Manifold difference or not?
                    e = [R*dX; dA] - z;
                    e(3) = wrapToPi(e(3));
                    
                    J_obs = [-R,            dR*dX;
                        zeros(1,2),   -1];
                    J_tar = [R,          zeros(2,1);
                        zeros(1,2), 1];
                    J = [J_obs, J_tar];
                    Hij = J'*info*J;
                    bij = e'*info*J;
                    
                    obs_istart = 3*N*(obs_t - 1) + 3*(obs_id-1) + 1;
                    tar_istart = 3*N*(tar_t - 1) + 3*(tar_id-1) + 1;
                    inds = [obs_istart:obs_istart + 2, tar_istart:tar_istart + 2];
                    H(inds,inds) = H(inds, inds) + Hij;
                    b(inds) = b(inds) + bij';
                    
                end
            end
            
            % Anchor robot 1 at t0 with fake reading
            % TODO: Better way of doing this?
            J0 = eye(3);
            info = 1E6*eye(3);
            e0 = zeros(3,1);
            H(1:3,1:3) = H(1:3,1:3) + J0'*info*J0;
            b(1:3) = b(1:3) + e0;
            
            % Solve for state vector increment
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
            
            % Store covariances
            allcovs = mat2cell(inv(H), 3*ones(N*T,1), 3*ones(N*T,1));
            locovs = cell(N, T);
            ci = sub2ind([N*T, N*T], 1:N*T, 1:N*T);
            locovs(1:N*T) = allcovs(ci);
            cov = locovs';                        
            
        end                
        
    end
    
end

