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
        
        function [] = Solve(obj, tol, max_iters)
            
            for i = 1:max_iters
                [dMax, dNorm] = obj.Iterate();
                fprintf(['Iteration: ', num2str(i), '\tDelta max: ', num2str(dMax), '\tDelta norm: ', num2str(dNorm), '\n']);
                if dNorm < tol
                    break
                end
            end            
            
        end
        
        function [dM, dN] = Iterate(obj)
            
            N = obj.truth.GetDimension();
            T = obj.truth.GetLength();                        
            
            anchor = obj.beliefs.states(1).poses(1);
            prev_seq = obj.beliefs.GetPosesDouble();
            
            sum_info = zeros(4,4,N,T); % Denominator - sum of weights
            sum_data = zeros(4,1,N,T); % Numerator - sum of weighted data
            
            pEsts = cell(N,T);
            
            % Iterate over time steps and measurements
            for t = 1:T                
                meas = obj.truth.states(t).measurements;    % Observations recorded at time t                
                M = numel(meas);                
                for m = 1:M
                   
                    meas_m = meas{m};
                    obs_t = meas_m.observer_time;
                    tar_t = meas_m.target_time;
                                       % TODO: Make overparameterized
                                       % orientation less ghetto!
                    obs_p = obj.beliefs.states(obs_t).poses(meas_m.observer_id);
                    tar_p = meas_m.ToPose(obs_p);    
                    pEsts{meas_m.target_id,tar_t} = [pEsts{meas_m.target_id,tar_t}, tar_p]; % For debugging
                    tar_pD = reshape(double(tar_p), 3, 1);
                    tar_pDO = [tar_pD(1:2); cos(tar_pD(3)); sin(tar_pD(3))];
                    meas_w = inv(meas_m.covariance);                    
                    meas_w(4,4) = meas_w(3,3);
                    
                    sum_info(:,:,meas_m.target_id,tar_t) = ...
                        sum_info(:,:,meas_m.target_id,tar_t) + meas_w;
                    sum_data(:,:,meas_m.target_id,tar_t) = ...
                        sum_data(:,:,meas_m.target_id,tar_t) + meas_w*tar_pDO;
                    
                    meas_m = meas_m.ToInverse();
                    obs_t = meas_m.observer_time;
                    tar_t = meas_m.target_time;
                                       
                    obs_p = obj.beliefs.states(obs_t).poses(meas_m.observer_id);
                    tar_p = meas_m.ToPose(obs_p);    
                    pEsts{meas_m.target_id,tar_t} = [pEsts{meas_m.target_id,tar_t}, tar_p]; % For debugging
                    tar_pD = reshape(double(tar_p), 3, 1);
                    tar_pDO = [tar_pD(1:2); cos(tar_pD(3)); sin(tar_pD(3))];
                    meas_w = inv(meas_m.covariance);
                    meas_w(4,4) = meas_w(3,3);
                    
                    sum_info(:,:,meas_m.target_id,tar_t) = ...
                        sum_info(:,:,meas_m.target_id,tar_t) + meas_w;
                    sum_data(:,:,meas_m.target_id,tar_t) = ...
                        sum_data(:,:,meas_m.target_id,tar_t) + meas_w*tar_pDO;
                                            
                end
                
            end
            
            % Iterate over time steps and robots
            for t = 1:T
               for n = 1:N
                  
                   data = sum_data(:,:,n,t);
                   info = sum_info(:,:,n,t);
                
                   weight = inv(info);
                   mle_data = reshape(weight*data, 1, 1, 4);
                   mle_p = Pose2D(mle_data(1:2), atan2(mle_data(4), mle_data(3)));
                   obj.beliefs.states(t).poses(n) = mle_p;
                   
               end
            end
            
            obj.beliefs.states(1).poses(1) = anchor;
            new_seq = obj.beliefs.GetPosesDouble();
            diff = abs(new_seq - prev_seq);
            dM = max(diff(:));
            dN = norm(diff(:));                                                            
            
        end
        
    end
    
end