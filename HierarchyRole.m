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
        beliefs_ind;
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
            
            obj.beliefs(100) = WorldState2D;
            obj.beliefs_ind = 1;
            
            obj.tx_buffer = BinBuffer(1, 50);
            
        end
        
        % TODO: Add AssignLeader method?
        function AssignFollowers(obj, followers)
            
            n = numel(followers);
            obj.followers = followers;
            
            obj.rx_buffer = BinBuffer(n, 50);
            
        end
        
        function [new] = Copy(obj)
            
            new = HierarchyRole(obj.level);
            new.solver = obj.solver.Copy();
            
            new.beliefs = obj.beliefs;
            new.beliefs_ind = obj.beliefs_ind;
            
            new.leader = obj.leader;
            new.followers = obj.followers;
            
        end
        
        % Return indices of team (leader, self, and co-members)
        function [ids] = GetTeam(obj)
            
            if isempty(obj.leader)
                ids = [];
                return
            end
            
            ids = [obj.leader.ownerID, obj.leader.followers.ownerID];
            
        end
        
        % Initialize agent's beliefs
        function Initialize(obj, state)
            
            obj.beliefs(1) = state;
            obj.beliefs_ind = 2;
            
        end
        
        % Inform this agent of new higher level beliefs
        function InformBeliefs(obj, beliefs, k)
            
            obj.higher_beliefs{k + 1} = beliefs;
            
        end
        
        % Transmit measurements to this agent
        % fid is the follower robot ownerID
        % TODO: How to deal with outages, multi-step bursts of
        % measurements?
        function InformMeasurements(obj, measurements, fid)
            
            ids = [obj.followers.ids];
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
            for i = 1:numel(measurements)
                z = measurements{i};
                if ~any(z.target_id == team_ids) || ~any(z.observer_id) == team_ids
                    obj.tx_buffer.Push(1,z);
                    measurements(i) = [];
                end
            end
            
            %2. Add new time slice to belief sequence
            new_state = obj.beliefs(obj.beliefs_ind - 1);
            new_state.measurements = measurements;
            new_state.time = new_state.time + obj.time_scale;
            obj.beliefs(obj.beliefs_ind) = obj.beliefs(obj.beliefs_ind - 1);
            obj.beliefs_ind = obj.beliefs_ind + 1;
            
            %3. Perform local optimization
            obj.beliefs = obj.solver.Solve(obj.beliefs);
            
            %4. Communicate results to followers
            for i = 1:numel(obj.followers)
                obj.followers(i).InformBeliefs(obj.beliefs, obj.level);
            end
            
            %5. If enough time passed, transmit buffer to leader
            obj.tstep_cnt = obj.tstep_cnt + 1;
            if obj.tstep_cnt >= obj.leader.time_scale/obj.time_scale;
                obj.leader.InformMeasurements(obj.tx_buffer.PopAll());
                obj.tstep_cnt = obj.tstep_cnt - 1;
            end
            
        end
        
    end
    
end