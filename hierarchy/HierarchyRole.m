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
        
        last_sent;
        
        tx_buffer;  % BinBuffer storing higher level measurements to send
        rx_buffer;  % BinBuffer that stores measurements from followers
        
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
            obj.last_sent = 0;
            
        end
        
        % Assigns followers and initializes relevant fields
        % Also assigns followers' leader field
        function AssignFollowers(obj, followers)
            
            n = numel(followers);
            obj.followers = followers;
            for i = 1:n
                obj.followers(i).leader = obj;
            end
            
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
        
        % Return times that this agent's local beliefs cover
        function [times] = GetGraphTimes(obj)
           
            times = [obj.local_beliefs.time];
            
        end
        
        % Queries position at time t relative to level k leader
        % k = 0 returns relative to root, k = -1 returns global estimate
        function [rel] = QueryRelation(obj, k, t)                        
            
            local_times = [obj.local_beliefs.time];
            if t < min(local_times) || t > max(local_times)
                error('Invalid relation query: local [%d, %d] query: %d\n', ...
                    min(local_times), max(local_times), t);
            end
            
            local_start_time = obj.higher_beliefs(end).target_time;
            local_rel = obj.ExtractLocalRelation(obj.ownerID, local_start_time, ...
                    obj.ownerID, t);                       
            if k < obj.level
                k_rel = obj.estimates(k+2);            
                rel = k_rel.Compose(local_rel);
            else
                rel = local_rel;
            end
            
        end
        
        function [rel] = QueryDisplacement(obj, tstart, tend)
           
            local_times = [obj.local_beliefs.time];
            if min(tstart, tend) < min(local_times) ...
                    || max(tstart, tend) > max(local_times)
                error('Invalid displacement query: local [%d, %d] query: %d %d\n', ...
                    min(local_times), max(local_times), tstart, tend);
            end
            
            rel = obj.ExtractLocalRelation(obj.ownerID, tstart, ...
                obj.ownerID, tend);
            
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
        function UpdateBeliefs(obj, beliefs)
            
            obj.higher_beliefs = beliefs;
            
            % Update local position estimates
            obj.BuildEstimates(obj.higher_beliefs);
            
            %Update local graph times
            obj.TrimGraph(obj.higher_beliefs(end).target_time);
            
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
            
            % Check target                                  
            if ~ismember(m.target_id, obj.GetTeam())
                % Find target's corresponding role agent and LCA
                other = FindLMR(obj, m.target_id);
                common = FindLCA(obj, other);
                
                % Determine proxies to translate to
                local_proxy = obj.GetAncestor(obj.level - common.level - 1);
                other_proxy = other.GetAncestor(other.level - common.level - 1);
                
                % Need to translate to nearest valid common time step
                % TODO: How to consolidate?
                %common_times = common.GetGraphTimes();
                common_obs_time = m.observer_time - mod(m.observer_time, common.time_scale);
                common_tar_time = m.target_time - mod(m.target_time, common.time_scale);
                
                local_to_proxy = obj.QueryRelation(local_proxy.level, m.observer_time);                
                other_to_proxy = other.QueryRelation(other_proxy.level, m.target_time);
                
                m = local_to_proxy.Compose(m);
                m = m.Compose(other_to_proxy.ToInverse());
                common.PushMeasurements(m);
                
            end
            
            
        end
        
        % Translates extra-team measurements and replaces old odometry
        % constraints with new ones when received. Returns intra-team
        % measurements and largest time value seen.
        function [measurements, latest_t] = ProcessMeasurements(obj, measurements)                        
            
            % Translate and transmit extra-team measurements
            team_ids = obj.GetTeam();
            remove = false(numel(measurements), 1);                        
            for i = 1:numel(measurements)
                z = measurements{i};
                if ~any(z.target_id == team_ids) || ~any(z.observer_id == team_ids)
                    obj.TranslateMeasurement(z); % Sends it to a better place
                    remove(i) = 1;
                end
            end
            measurements(remove) = [];
            
            % Determine new times
            latest_t = -Inf;
            for i = 1:numel(measurements)
                z = measurements{i};
                latest_t = max([z.target_time, z.observer_time, latest_t]);
            end
            
            % Replace old odometry constraints with new ones
            remove = false(numel(measurements), 1);
            [~, tMap] = obj.local_beliefs.BuildMaps();
            for i = 1:numel(measurements)
                z = measurements{i};                
                if z.observer_time == z.target_time
                    continue;                    
                end
                remove(i) = 1;
                obs_t = tMap.Forward(z.observer_time);
                old_zs = obj.local_beliefs(obs_t).measurements;
                for j = 1:numel(old_zs);
                   zo = old_zs{j};
                   if z.SameRelation(zo)
                      obj.local_beliefs(obs_t).measurements{j} = z;
                      break; % Should only be one corresponding
                   end
                end
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
            [measurements, latest_t] = obj.ProcessMeasurements(measurements);                       
            
            % Add new time slice to belief sequence if needed
            obj.ExtendGraph(latest_t);            
            [~, tMap] = obj.local_beliefs.BuildMaps();
            
            % Add measurements to corresponding states
            for i = 1:numel(measurements)
                z = measurements{i};
                ind = tMap.Forward(z.observer_time);
                % Note that WorldState2D is a value class, but we don't
                % have to reassign b/c we don't copy it here
                obj.local_beliefs(ind).measurements{end+1} = z;                
            end
            
            % Perform local optimization
            fprintf('Optimizing at ID: %d, k: %d, t: %d\n', obj.ownerID, ...
                obj.level, obj.time)
            [obj.local_beliefs, obj.beliefs_cov] = obj.solver.Solve(obj.local_beliefs);
            
            % Inform leader of new local movement
            start_t = obj.local_beliefs(1).time;
            end_t = obj.local_beliefs(end).time;
            if ~isempty(obj.leader)
                sub_times = start_t:obj.leader.time_scale:end_t;
                meas = cell(numel(sub_times) - 1, 1);
                for i = 1:numel(sub_times)-1                    
                    op_z = obj.ExtractLocalRelation(obj.ownerID, sub_times(i), ...
                        obj.ownerID, sub_times(i+1));
                    meas{i} = op_z;                    
                end
                obj.leader.PushMeasurements(meas);                
            end
            
            % Communicate only at fixed intervals
%             if mod(obj.time, obj.time_scale) ~= 0
%                 obj.time = obj.time + 1;
%                 return
%             end
            
            % Inform followers of latest local_beliefs
            if isempty(obj.followers)
                obj.time = obj.time + 1;
                return
            end
                        
            fprintf('Informing at ID: %d, k: %d, t: %d\n', obj.ownerID, ...
                obj.level, obj.time)
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                f_bel = obj.ExtractLocalRelation(obj.ownerID, start_t, ...
                    f.ownerID, end_t);
                obj.followers(i).UpdateBeliefs([obj.higher_beliefs, f_bel]);
            end
            
            obj.time = obj.time + 1;
            
        end
        
    end
    
    methods(Access = private)
        
        % Builds this agent's root position in all higher reference frames
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
        
        % Trims the local graph to start from root_time
        function TrimGraph(obj, root_time)
            
            [~, tMap] = obj.local_beliefs.BuildMaps();
                        
            % "Marginalize" past information into relations at new time
            % TODO: This doesn't seem right. Should instead translate +
            % compress old measurements into the new frame
            marginalized_rel = cell(numel(obj.followers) + 1, 1);
            for i = 1:numel(obj.followers)
                f_id = obj.followers(i).ownerID;
                rel = obj.ExtractLocalRelation(obj.ownerID, root_time, ...
                    f_id, root_time);
                marginalized_rel{i} = rel;
            end
            
            root_ind = tMap.Forward(root_time);
            obj.local_beliefs = obj.local_beliefs(root_ind:end);
            obj.local_beliefs(1).measurements = ...
                [obj.local_beliefs(1).measurements, marginalized_rel{i}];
            
        end
        
        % Extends the local graph to end at end_time
        % States are initialized to the last valid belief
        function ExtendGraph(obj, end_time)
           
            last_valid = obj.local_beliefs(end);
            new_times = last_valid.time:obj.time_scale:end_time;
            for i = 2:numel(new_times)
               
                new_state = last_valid; % Creates a copy
                new_state.measurements = {};
                new_state.time = new_times(i);
                obj.local_beliefs(end+1) = new_state;
                
            end
            
        end
        
    end
    
end