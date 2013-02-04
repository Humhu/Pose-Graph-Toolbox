% A hierarchical role object
% Contains all information required to perform role's duties
classdef HierarchyRole < handle
    
    properties
        
        level;      % Rank of role (root level = 0)
        time_scale; % Time step size
        
        ownerID;         % ownerID of corresponding robot
        leader;     % Reference to leader HierarchyRole object
        followers;  % Array of follower HierarchyRole objects
        
        higher_beliefs = {}; % WorldState2D arrays representing higher beliefs
        
        beliefs = WorldState2D; % WorldState2D array representing local beliefs   
        beliefs_cov;    % Covariance matrix
        solver;     % GNSolver object
        
        tstep_cnt;  % Time steps since last transmit
        tx_buffer;  % BinBuffer storing higher level measurements to send
        rx_buffer;  % BinBuffer that stores measurements from followers
        
    end
    
    methods
        
        function [obj] = HierarchyRole(level)
            
            obj.level = level;
            obj.solver = GNSolver(1E-3, 100);
            
            obj.higher_beliefs = cell(1,level);                                                
            obj.tx_buffer = BinBuffer(1, 50);
            
            obj.tstep_cnt = 0;
            
        end
        
        % TODO: Add AssignLeader method?
        function AssignFollowers(obj, followers)
            
            n = numel(followers);
            obj.followers = followers;
            for i = 1:n
                obj.followers(i).leader = obj;
            end
            obj.rx_buffer = BinBuffer(n, 50);
            
        end
        
        function [new] = Copy(obj)
            
            new = HierarchyRole(obj.level);
            new.solver = obj.solver.Copy();
            
            new.beliefs = obj.beliefs;            
            new.beliefs_cov = obj.beliefs_cov;
            
            new.leader = obj.leader;
            new.followers = obj.followers;
            
        end
        
        % Return indices of team (self and followers)
        function [ids] = GetTeam(obj)
            
            ids = obj.ownerID;
            
            if isempty(obj.followers)                
                return
            end
            
            % Self ID is contained in followers.ownerID
            ids = [obj.followers.ownerID];
            
        end
        
        % Initialize agent's beliefs
        % TODO: How to initialize position guesses?
        function Initialize(obj, state)           
            
            obj.beliefs(1) = state;
            obj.tstep_cnt = 1;
            % TODO: Process instead of tossing measurements?
            %obj.beliefs(1).measurements = [];
            
        end
        
        % Inform this agent of new higher level beliefs
        function InformBeliefs(obj, beliefs, k)
            
            obj.higher_beliefs{k + 1} = beliefs;
            
        end
        
        % Converts local beliefs into proper time scale
        % We only send the optimized results for the group leader (i = 1)
        function [z] = ConvertOptimization(obj)
            curr_state = obj.beliefs(end);
            prev_state = obj.beliefs(end - obj.leader.time_scale);            
            curr_pose = curr_state.poses(:,1);
            prev_pose = prev_state.poses(:,1);
            
            N = numel(obj.GetTeam());
            
            z = MeasurementRelativePose(curr_pose, prev_pose, zeros(3));
            z.observer_id = obj.ownerID;
            z.target_id = obj.ownerID;
            z.observer_time = curr_state.time;
            z.target_time = prev_state.time;
            z.covariance = obj.beliefs_cov{end - N + 1, ...
                end - N - obj.leader.time_scale + 1};
            
        end
        
        % Transmit measurements to this agent
        % fid is the follower robot ownerID
        % TODO: How to deal with outages, multi-step bursts of
        % measurements?
        function InformMeasurements(obj, measurements, fid)
            
            ids = [obj.followers.ownerID];
            obj.rx_buffer.Push(ids == fid, measurements);
            
            % When all follower readings received, process
            if ~any(obj.rx_buffer.IsEmpty())
                obj.ProcessMeasurements(obj.rx_buffer.PopAll());
            end
            
        end
        
        % Process new batch of measurements
        % Should end up being called every obj.time_scale steps
        function ProcessMeasurements(obj, measurements)
            
            %1. Buffer away non-local measurements
            team_ids = obj.GetTeam();           
            remove = false(numel(measurements), 1);
            for i = 1:numel(measurements)
                z = measurements{i};
                if ~any(z.target_id == team_ids) || ~any(z.observer_id == team_ids)
                    obj.tx_buffer.Push(1,z);
                    remove(i) = 1;
                end
            end
            measurements(remove) = [];
            
            %2. Add new time slice to belief sequence
            new_state = obj.beliefs(end);
            new_state.measurements = measurements;
            new_state.time = new_state.time + obj.time_scale;
            obj.beliefs(end + 1) = new_state;
            
            %3. Perform local optimization
            [obj.beliefs, obj.beliefs_cov] = obj.solver.Solve(obj.beliefs);
            
            %4. Communicate results to followers
            for i = 1:numel(obj.followers)
                obj.followers(i).InformBeliefs(obj.beliefs, obj.level);
            end
            
            %5. If enough time passed, transmit buffer to leader
            obj.tstep_cnt = obj.tstep_cnt + 1;
            
            if isempty(obj.leader)
                continue
            end
            
            if obj.tstep_cnt > obj.leader.time_scale;
                op_z = obj.ConvertOptimization();
                meas = [obj.tx_buffer.PopAll(), {op_z}];
                obj.leader.InformMeasurements(meas, obj.ownerID);
                obj.tstep_cnt = 0; % Because we popped all
            end
            
        end
        
    end
    
end