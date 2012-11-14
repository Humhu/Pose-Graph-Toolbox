% Represents an iterative localization problem
classdef EMIterate < handle
    
    properties
        
        beliefs;        % Trajectory beliefs
        truth;          % Observations and ground truth
        
    end
    
    methods
        
        function obj = EMIterate(N)
            
            if nargin == 0
                return
            end
            
            obj.beliefs = Sequence2D(N);
            obj.truth = Sequence2D(N);
            
        end
        
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
            % TODO: Do!
            
        end
        
        function [] = Update(obj, state)
            
           obj.truth.Write(state);
           
           % Use previous state as guess for next one
           lastBelief = obj.beliefs.states(obj.beliefs.GetLength());
           lastBelief.time = state.time;     
           lastBelief.measurements = state.measurements;
           obj.beliefs.Write(lastBelief); 
            
        end
        
        function [] = Solve(obj, tol, max_iters, vis_on)
            
            for i = 1:max_iters
                obj.Iterate();
                if vis_on
                    obj.Visualize();
                end
            end
            obj.Visualize();
            
        end
        
        function [] = Iterate(obj)
            
            N = obj.truth.GetDimension();
            T = obj.truth.GetLength();                        
            
            anchor = obj.beliefs.states(1).poses(1);
            prev_seq = obj.beliefs.GetPosesDouble();
            
            sum_info = zeros(3,3,N,T); % Denominator - sum of weights
            sum_data = zeros(3,1,N,T); % Numerator - sum of weighted data
            
            % Iterate over time steps and measurements
            for t = 1:T                
                meas = obj.truth.states(t).measurements;    % Observations recorded at time t                
                M = numel(meas);                
                for m = 1:M
                   
                    meas_m = meas{m};
                    obs_t = meas_m.observer_time;
                    tar_t = meas_m.target_time;
                                       
                    obs_p = obj.beliefs.states(obs_t).poses(meas_m.observer_id);
                    tar_p = meas_m.ToPose(obs_p);    
                    tar_pD = reshape(double(tar_p), 3, 1);
                    meas_w = inv(meas_m.covariance);
                    
                    sum_info(:,:,meas_m.target_id,tar_t) = ...
                        sum_info(:,:,meas_m.target_id,tar_t) + meas_w;
                    sum_data(:,:,meas_m.target_id,tar_t) = ...
                        sum_data(:,:,meas_m.target_id,tar_t) + meas_w*tar_pD;
                    
                    meas_m = meas_m.ToInverse();
                    obs_t = meas_m.observer_time;
                    tar_t = meas_m.target_time;
                                       
                    obs_p = obj.beliefs.states(obs_t).poses(meas_m.observer_id);
                    tar_p = meas_m.ToPose(obs_p);    
                    tar_pD = reshape(double(tar_p), 3, 1);
                    meas_w = inv(meas_m.covariance);
                    
                    sum_info(:,:,meas_m.target_id,tar_t) = ...
                        sum_info(:,:,meas_m.target_id,tar_t) + meas_w;
                    sum_data(:,:,meas_m.target_id,tar_t) = ...
                        sum_data(:,:,meas_m.target_id,tar_t) + meas_w*tar_pD;
                                            
                end
                
            end
            
            % Iterate over time steps and robots
            for t = 1:T
               for n = 1:N
                  
                   data = sum_data(:,:,n,t);
                   info = sum_info(:,:,n,t);
                
                   weight = inv(info);
                   mle_data = reshape(weight*data, 1, 1, 3);
                   mle_p = Pose2D(mle_data(1:2), mle_data(3));
                   obj.beliefs.states(t).poses(n) = mle_p;
                   
               end
            end
            
            obj.beliefs.states(1).poses(1) = anchor;
            new_seq = obj.beliefs.GetPosesDouble();
            diff = abs(new_seq - prev_seq);
            diff_max = max(diff(:));
            diff_norm = norm(diff(:));
                        
            fprintf(['Diff max: ', num2str(diff_max), ', Diff norm: ', num2str(diff_norm), '\n']);
            
            % TODO: Determine convergence criteria
            
        end
        
    end
    
end