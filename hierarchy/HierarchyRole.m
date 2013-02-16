% A hierarchical role object
% Contains all information required to perform role's duties
classdef HierarchyRole < handle
    
    properties
        
        level;      % Rank of role (root level = 0)
        time;       % Internal time counter
        time_scale; % Time step size
        
        ownerID;         % ownerID of corresponding robot
        leader;     % Reference to leader HierarchyRole object
        followers;  % Array of follower HierarchyRole objects
        
        estimates;
        higher_beliefs; % WorldState2D array representing higher beliefs
        % Each WorldState2D at index i corresponds to the relative pose of
        % the ancestor at level (i + 1) wrt ancestor at level (i).
        
        beliefs = WorldState2D; % WorldState2D array representing local beliefs
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
        
        function [ids] = GetAncestors(obj)
            
            ids = obj.GetTeam();
            for i = 1:numel(obj.followers)
                ids = [ids, obj.followers(i).GetAncestors()];
            end
            ids = unique(ids);
            
        end
        
        function [ids] = GetDescendants(obj)
            
        end
        
        % Initialize agent's beliefs
        function Initialize(obj, state)
            
            ids = obj.GetTeam();
            substate = state;
            substate.poses = substate.poses(:,ismember(substate.ids, ids));
            substate.ids = ids;
            substate = substate.Zero();
            substate.measurements = {}; % Do we want this?
            obj.beliefs = substate;
            N = substate.GetDimension();
            obj.beliefs_cov = 1E-6*eye(3*N, 3*N);
            
            obj.time = 0;
            
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                f.Initialize(state);                                                
            end
            
            if obj.level ~= 0
                return
            end
            
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
            
            %TODO: Update local graph times
            obj.TrimGraph(beliefs(end).target_time);
            
            % Pass down to followers
            for i = 1:numel(obj.followers)
                f = obj.followers(i);
                s_time = obj.beliefs(1).time;
                e_time = obj.beliefs(end).time;
                z = obj.ExtractLocalRelation(obj.ownerID, s_time, ...
                    f.ownerID, e_time);
                f.UpdateBeliefs([obj.higher_beliefs, z]);
            end
            
        end
        
        % Trims the local graph to start from start_time
        function TrimGraph(obj, start_time)
            
            
            
        end
        
        % Translates an extra-group measurement to the appropriate frame
        % and level
        function [m_trans] = TranslateMeasurement(obj, m)
            %TODO: IMPLEMENT ME!
            locals = obj.GetTeam();
            
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
            
            obj.time = obj.time + 1;
            
            % If there are no new measurements, there's nothing to do!
            if obj.rx_buffer.IsEmpty()
                return
            end
            
            measurements = obj.rx_buffer.PopAll();
            
            % Process measurements
            local_times = [obj.beliefs.time];                                    
            start_t = local_times(1);
            end_t = local_times(end);
            input_times = [];
            obs_times = [];
            team_ids = obj.GetTeam();
            remove = false(numel(measurements), 1);
            
            for i = 1:numel(measurements)
                z = measurements{i};                
                if ~any(z.target_id == team_ids) || ~any(z.observer_id == team_ids)
                    z = obj.TranslateMeasurement(z); %Think about this...
                    obj.tx_buffer.Push(1,z);
                    remove(i) = 1;
                else
                    input_times = [input_times, z.observer_time, z.target_time];
                    obs_times = [obs_times, z.observer_time];
                end
            end
            
            % Remove buffered measurements and find new times
            measurements(remove) = [];
            input_times = unique(input_times); % Also sorts ascending
            new_times = input_times(input_times > start_time);
            
            % Add new time slice to belief sequence if needed
            for i = 1:numel(new_times)
                t = new_times(i);
                new_state = obj.beliefs(end);                
                new_state.measurements = measurements(obs_times == t);
                new_state.time = t;
                obj.beliefs = [obj.beliefs, new_state];
            end
                
            %3. Perform local optimization if graph updated
            [obj.beliefs, obj.beliefs_cov] = obj.solver.Solve(obj.beliefs);
            
            % Communicate only at fixed intervals
            if mod(obj.time, obj.time_scale) ~= 0
                return
            end
            
            % Inform followers of latest beliefs
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
            
        end
        
    end
    
    methods(Access = private)
        
        % Builds this agent's positions in all higher reference frames
        % given a chain of relative measurements. The chain should be
        % ordered with measurements(1) corresponding to root->f1, and
        % measurements(end) corresponding to fn->obj
        function BuildEstimates(obj, measurements)
            
            if numel(measurements) ~= obj.level
                error(['Wrong number of measurements given to BuildEstimates: ', ...
                    'level: %d, #meas: %d'], obj.level, numel(measurements));
            end
            
            % Initialize all as last link
            for i = 1:obj.level
                obj.estimates(i) = measurements(end);
            end
            
            % Build chain
            for i = obj.level-1:-1:1
                m = measurements(i);
                obj.estimates(i) = m.Compose(obj.estimates(i));
            end
            
        end
        
        % Extracts a specified relation from the local optimized graph
        function [z] = ExtractLocalRelation(obj, start_id, start_time, ...
                end_id, end_time)
            
            [idMap, tMap] = obj.beliefs.BuildMaps();
            
            s_id = idMap.Forward(start_id);
            s_time = tMap.Forward(start_time);
            e_id = idMap.Forward(end_id);
            e_time = tMap.Forward(end_time);
            
            start_pose = obj.beliefs(s_time).poses(:,s_id);
            end_pose = obj.beliefs(e_time).poses(:,e_id);
            
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
        
    end
    
end