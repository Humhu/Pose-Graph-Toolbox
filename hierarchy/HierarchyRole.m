% A hierarchical role object
% Contains all information required to perform role's duties
classdef HierarchyRole < handle
    
    properties
        
        level;      % Rank of role (root level = 0)
        time;       % Internal time counter
        time_scale; % Time step size
        
        ownerID;    % ownerID of corresponding robot
        leader;     % Reference to leader HierarchyRole object
        followers;  % Array of follower HierarchyRole objects
        
        estimates;  % MeasurementRelativePose array representing position
        % estimates relative to ancestors.
        higher_beliefs; % MeasurementRelativePose array representing ancestor beliefs
        % Each Measurement at index i corresponds to the relative pose of
        % the ancestor at level (i + 1) wrt ancestor at level (i).
        
        local_beliefs = WorldState2D; % WorldState2D array representing local beliefs
        beliefs_cov;    % Covariance matrix
        solver;     % GNSolver object
        
        tx_buffer;  % BinBuffer storing higher level measurements to send
        rx_buffer;  % BinBuffer that stores measurements from followers
        heard_from;
        
    end
    
    methods
        
        function [obj] = HierarchyRole(level)
            
            obj.level = level;
            obj.solver = GNSolver(1E-3, 100);
            
            obj.higher_beliefs = MeasurementRelativePose.empty(1, 0);
            obj.estimates = MeasurementRelativePose.empty(1,0);
            obj.tx_buffer = BinBuffer(1, 50);
            obj.rx_buffer = BinBuffer(1, 50);
            obj.time = 0;
            
        end
        
        % Assigns followers and initializes relevant fields
        % Also assigns followers' leader field
        function AssignFollowers(obj, followers)
            
            n = numel(followers);
            obj.followers = followers;
            for i = 1:n
                obj.followers(i).leader = obj;
            end
            obj.heard_from = false(n,1);
            
        end
        
        % Creates a shallow copy of this HierarchyRole object
        function [new] = Copy(obj)
            
            new = HierarchyRole(obj.level);
            new.solver = obj.solver.Copy();
            
            new.local_beliefs = obj.local_beliefs;
            new.beliefs_cov = obj.beliefs_cov;
            
            new.leader = obj.leader;
            new.followers = obj.followers;
            
        end
        
        % TODO: Clean up naming for these Get/Traverse type methods
        % Return indices of team (self and followers)
        function [ids] = GetTeam(obj)
            
            ids = obj.ownerID;
            
            if isempty(obj.followers)
                return
            end
            
            % Self ID is contained in followers.ownerID
            ids = [obj.followers.ownerID];
            
        end
        
        % Get IDs of all ancestors FIX
        function [ids] = GetDescendants(obj)
            
            ids = obj.GetTeam();
            for i = 1:numel(obj.followers)
                ids = [ids, obj.followers(i).GetDescendants()];
            end
            ids = unique(ids);
            
        end
        
        % Return ancestor k levels above
        function [anc] = GetAncestor(obj, k)
        
            anc = obj;
            for i = 1:k
               anc = anc.leader; 
            end
            
        end
        
        % Queries position relative to level k leader
        % k = 0 returns relative to root, k = -1 returns global estimate
        function [rel] = QueryRelation(obj, k)
            
            if k == obj.level
                rel = MeasurementRelativePose(zeros(3,1), zeros(3,1), zeros(3));
                rel.observer_id = obj.ownerID;
                rel.observer_time = obj.time;
                rel.target_id = obj.ownerID;
                rel.target_time = obj.time;
                rel.covariance = zeros(3);
            else
                rel = obj.estimates(k+2);
            end
            
        end
            
        % Initialize agent's local_beliefs
        function Initialize(obj, state)
            
            ids = obj.GetTeam();
            substate = state;
            substate.poses = substate.poses(:,ismember(substate.ids, ids));
            substate.ids = ids;
            substate = substate.Zero(); % Leader is at origin
            substate.measurements = {}; % Do we want this?
            obj.local_beliefs = substate;
            N = substate.GetDimension();
            obj.beliefs_cov = 1E-6*eye(3*N, 3*N); % TODO: Hard-coded initialization prior
            
            obj.time = 0;
            
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                f.Initialize(state);
            end
            
            if obj.level ~= 0
                return
            end
            
            global_pose = state.poses(:,1);
            global_belief = MeasurementRelativePose(zeros(3,1), global_pose, zeros(3));
            global_belief.observer_id = -1;
            global_belief.observer_time = 0;
            global_belief.target_id = obj.ownerID;
            global_belief.target_time = 0;
            global_belief.covariance = 1E-6*eye(3);
            
            obj.UpdateBeliefs(global_belief);
            
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                f_bel = obj.ExtractLocalRelation(obj.ownerID, 0, ...
                    f.ownerID, 0);
                obj.followers(i).UpdateBeliefs([obj.higher_beliefs, f_bel]);
            end
            
        end
        
        % Informs the agent of updated higher-level belief states
        % Beliefs should be an ordered array of measurements
        function UpdateBeliefs(obj, local_beliefs)
            
            obj.higher_beliefs = local_beliefs;
            
            % Update local position estimates
            obj.BuildEstimates(obj.higher_beliefs);
            
            %TODO: Update local graph times
            obj.TrimGraph(local_beliefs(end).target_time);
            
            % Pass down to followers
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                s_time = obj.local_beliefs(1).time;
                e_time = obj.local_beliefs(end).time;
                z = obj.ExtractLocalRelation(obj.ownerID, s_time, ...
                    f.ownerID, e_time);
                f.UpdateBeliefs([obj.higher_beliefs, z]);
            end
            
        end
        
        % Translates an extra-group measurement to the appropriate frame
        % and level
        % Currently we assume that only the target ID can be extra-group
        function TranslateMeasurement(obj, m)
            
            local_ids = obj.GetTeam();
            m
            % Check target
            if ~ismember(m.target_id, local_ids)
                % Find target's corresponding role agent and LCA
                other = FindLMR(obj, m.target_id);
                common = FindLCA(obj, other);
                
                % Determine proxies to translate to
                local_proxy = obj.GetAncestor(obj.level - common.level - 1);
                other_proxy = other.GetAncestor(other.level - common.level - 1);
                
                local_relation = obj.QueryRelation(local_proxy.level);
                other_relation = other.QueryRelation(other_proxy.level);
                
                m = local_relation.Compose(m);
                m = m.Compose(other_relation.ToInverse());
                common.PushMeasurements(m);
                
            end
            
            
        end
        
        % Transmit measurements to this agent
        % fid is the follower robot ownerID
        function InformMeasurements(obj, measurements, fid)
            
            ids = [obj.followers.ownerID];
            obj.heard_from(ids == fid) = 1;
            obj.rx_buffer.Push(1, measurements);
            
            % When all follower readings received, process
            if all(obj.heard_from)
                obj.PushMeasurements(obj.rx_buffer.PopAll());
                obj.ProcessMeasurements();
                obj.heard_from = false(size(obj.heard_from));
            end
            
        end
        
        % Input measurements into this agent for processing, whether from a
        % robot or follower agent
        function PushMeasurements(obj, measurements)
            
            obj.rx_buffer.Push(1, measurements);
            
        end
        
        % Process internal functions. Should be called once every time
        % step.
        function Step(obj)                        
            
            % If there are no new measurements, there's nothing to do!
            if obj.rx_buffer.IsEmpty()
                obj.time = obj.time + 1;
                return
            end
            
            measurements = obj.rx_buffer.PopAll();
            
            % Process measurements
            local_times = [obj.local_beliefs.time];
            start_t = local_times(1);
            end_t = local_times(end);
            input_times = [];
            obs_times = [];
            team_ids = obj.GetTeam();
            remove = false(numel(measurements), 1);
            
            for i = 1:numel(measurements)
                z = measurements{i};
                if ~any(z.target_id == team_ids) || ~any(z.observer_id == team_ids)
                    obj.TranslateMeasurement(z); % Sends it to a better place
                    %obj.tx_buffer.Push(1,z);
                    remove(i) = 1;
                else
                    input_times = [input_times, z.observer_time, z.target_time];
                    obs_times = [obs_times, z.observer_time];
                end
            end
            
            % Remove buffered measurements and find new times
            % TODO: Fix!
            measurements(remove) = [];
            input_times = unique(input_times); % Also sorts ascending
            new_times = input_times(input_times > start_time);
            
            % Add new time slice to belief sequence if needed
            for i = 1:numel(new_times)
                t = new_times(i);
                new_state = obj.local_beliefs(end);
                new_state.measurements = measurements(obs_times == t);
                new_state.time = t;
                obj.local_beliefs = [obj.local_beliefs, new_state];
            end
            
            %3. Perform local optimization if graph updated
            [obj.local_beliefs, obj.beliefs_cov] = obj.solver.Solve(obj.local_beliefs);
            
            % Communicate only at fixed intervals
            if mod(obj.time, obj.time_scale) ~= 0
                obj.time = obj.time + 1;
                return
            end
            
            % Inform followers of latest local_beliefs
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                f_bel = obj.ExtractLocalRelation(obj.ownerID, start_t, ...
                    f.ownerID, end_t);
                obj.followers(i).UpdateBeliefs([obj.higher_beliefs, f_bel]);
            end
            
            % Inform leader of new measurements
            if ~isempty(obj.leader)
                op_z = obj.ExtractLocalRelation(obj.ownerID, start_t, ...
                    obj.ownerID, end_t);
                meas = [obj.tx_buffer.PopAll(), {op_z}];
                obj.leader.InformMeasurements(meas, obj.ownerID);
            end
            
            obj.time = obj.time + 1;
            
        end
        
    end
    
    methods(Access = private)
        
        % Builds this agent's positions in all higher reference frames
        % given a chain of relative measurements. The chain should be
        % ordered with measurements(1) corresponding to root->f1, and
        % measurements(end) corresponding to fn->obj
        function BuildEstimates(obj, measurements)
            
            if numel(measurements) ~= obj.level + 1
                error(['Wrong number of measurements given to BuildEstimates: ', ...
                    'level: %d, #meas: %d'], obj.level, numel(measurements));
            end
            
            % Initialize all as last link
            for i = 1:obj.level+1
                obj.estimates(i) = measurements(end);
            end
            
            % Build chain by composing measurements in reverse-order
            for i = obj.level:-1:1
                m = measurements(i);
                for j = i:-1:1
                    obj.estimates(j) = m.Compose(obj.estimates(j));
                end
            end
            
        end
        
        % Extracts a specified relation from the local optimized graph
        function [z] = ExtractLocalRelation(obj, start_id, start_time, ...
                end_id, end_time)
            
            [idMap, tMap] = obj.local_beliefs.BuildMaps();
            
            s_id = idMap.Forward(start_id);
            s_time = tMap.Forward(start_time);
            e_id = idMap.Forward(end_id);
            e_time = tMap.Forward(end_time);
            
            start_pose = obj.local_beliefs(s_time).poses(:,s_id);
            end_pose = obj.local_beliefs(e_time).poses(:,e_id);
            
            z = MeasurementRelativePose(start_pose, end_pose, zeros(3));
            z.observer_id = start_id;
            z.observer_time = start_time;
            z.target_id = end_id;
            z.target_time = end_time;
            
            N = numel(obj.GetTeam());
            
            prev_start = 3*N*(s_time - 1) + (s_id - 1) + 1;
            prev_end = prev_start + 2;
            curr_start = 3*N*(e_time - 1) + (e_id - 1) + 1;
            curr_end  = curr_start + 2;
            inds = [prev_start:prev_end, curr_start:curr_end];
            
            fullcov = obj.beliefs_cov(inds, inds);
            b = [-eye(3), eye(3)];
            z.covariance = b*fullcov*b';
            
        end
        
        % Trims the local graph to start from start_time
        function TrimGraph(obj, start_time)
            
            [~, tMap] = obj.local_beliefs.BuildMaps();
            newStart = tMap.Forward(start_time);
            obj.local_beliefs = obj.local_beliefs(newStart:end);
            
        end
        
    end
    
end