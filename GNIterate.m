% Represents an iterative localization problem
classdef GNIterate < handle
    
    properties
        
        beliefs;
        truth;
        covariance;
        
    end
    
    methods
        
        function obj = GNIterate(N)
            
            if nargin == 0
                return
            end
            
            obj.beliefs = Sequence2D(N);
            obj.truth = Sequence2D(N);
            obj.covariance = {};
            
        end
        
        % TODO: Split truth/beliefs initialization
        function [] = Initialize(obj, state)
            
            if nargin == 2
                obj.truth.Clear();
                obj.truth.Write(state);
                obj.beliefs.Clear();
                obj.beliefs.Write(state);
            end
            
            % Else make beliefs random
            N = obj.truth.GetDimension();
            T = obj.truth.GetLength();
            % TODO: Do!!
            
        end
        
        function [] = Update(obj, state)
            
            obj.truth.Write(state);
            
            % Use previous state as guess for next one
            lastBelief = obj.beliefs.states(obj.beliefs.GetLength());
            lastBelief.time = state.time;
            lastBelief.measurements = state.measurements;
            obj.beliefs.Write(lastBelief);
            
        end
        
        function [] = Solve(obj, tol, max_iters)
            
            for i = 1:max_iters
                [dMax, dNorm] = obj.Iterate();
                fprintf(['Iteration: ', num2str(i), '\tDelta max: ', num2str(dMax), '\tDelta norm: ', num2str(dNorm), '\n']);
                if dNorm < tol
                    break
                end
            end
            
        end
        
        % Relative Pose Iteration
        function [dM, dN] = Iterate(obj)
            
            N = obj.beliefs.GetDimension();
            T = obj.beliefs.GetLength();
            
            anchor = obj.beliefs.states(1).poses(:,1);
            prev_seq = obj.beliefs.GetPosesDouble();
            
            % System Jacobian matrix
            H = zeros(3*N*T, 3*N*T);
            b = zeros(3*N*T, 1);
            
            for t = 1:T
                % TODO: Move measurements to beliefs?
                % TODO: Support non-ordered IDs w/ index mapping
                meas_t = obj.truth.states(t).measurements;
                M = numel(meas_t);
                for i = 1:M
                    m = meas_t{i};
                    obs_t = m.observer_time;
                    tar_t = m.target_time;
                    obs_id = m.observer_id;
                    tar_id = m.target_id;
                    
                    obs_p = obj.beliefs.states(obs_t).poses(:, obs_id);
                    tar_p = obj.beliefs.states(tar_t).poses(:, tar_id);
                    
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
            J0 = eye(3);
            info = 1E6*eye(3);
            e0 = zeros(3,1);
            H(1:3,1:3) = H(1:3,1:3) + J0'*info*J0;
            b(1:3) = b(1:3) + e0;
            
            dX = H\-b;
            
            for t = 1:T
                for n = 1:N
                    pos = obj.beliefs.states(t).poses(:,n);
                    inds = 3*N*(t - 1) + 3*(n - 1)  + 1;
                    pos = pos + dX(inds:inds + 2);
                    pos(3) = wrapToPi(pos(3));
                    obj.beliefs.states(t).poses(:,n) = pos;
                end
            end
            
            % Rotate so that anchor is stationary
%             at = obj.beliefs.states(1).poses(:,1);
%             RT = anchor - at;
%             da = wrapToPi(RT(3));
%             Rc = [cos(da), sin(da);
%                   -sin(da), cos(da)];
%             trans = RT(1:2);
%             d = obj.beliefs.GetPosesDouble();
%             for t = 1:T
%                 p = d(3*t-2:3*t-1,:);
%                 p = bsxfun(@plus, Rc*bsxfun(@minus, p, at(1:2)), at(1:2));
%                 p = bsxfun(@plus, p, trans);
%                 a = wrapToPi(d(3*t,:) + da);
%                 obj.beliefs.states(t).poses = [p; a];                
%             end
            
            % Store covariances
            allcovs = mat2cell(inv(H), 3*ones(N*T,1), 3*ones(N*T,1));
            locovs = cell(N, T);
            ci = sub2ind([N*T, N*T], 1:N*T, 1:N*T);
            locovs(1:N*T) = allcovs(ci);
            obj.covariance = locovs';
            
            diff = abs(obj.beliefs.GetPosesDouble() - prev_seq);
            dM = max(diff(:));
            dN = rms(diff(:));
            
        end
        
        % Range-Bearing Iteration
        % TODO: Update to be consistent with Sequence2Ds
        function [dM, dN] = IterateRB(obj)
            
            % Calculate errors
            N = numel(obj.pose_beliefs);
            errors = zeros(N,N,2);
            
            num_rels = size(obj.meas, 2);
            
            for k = 1:num_rels
                
                z = obj.meas{k};
                r = z.range;
                a = z.bearing;
                i = z.observer_id;
                j = z.target_id;
                
                pi = obj.pose_beliefs(i);
                pix = pi.position(1);
                piy = pi.position(2);
                pit = pi.orientation;
                pj = obj.pose_beliefs(j);
                pjx = pj.position(1);
                pjy = pj.position(2);
                
                ex = pjx - pix - r*cos(double(a + pit));
                ey = pjy - piy - r*sin(double(a + pit));
                
                errors(i,j,1) = ex;
                errors(i,j,2) = ey;
                
            end
            
            % Generate Jacobian
            H = zeros(3*N);
            b = zeros(3*N,1);
            
            for k = 1:num_rels
                
                z = obj.meas{k};
                r = z.range;
                a = z.bearing;
                i = z.observer_id;
                j = z.target_id;
                
                Jij = zeros(2, 3*N);
                
                pi = obj.pose_beliefs(i);
                ti = pi.orientation;
                
                info = obj.meas{i}.covariance^-1;
                
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
        
        
        
    end
    
end