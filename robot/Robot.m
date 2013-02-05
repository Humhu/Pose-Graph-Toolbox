% Represents a robot and its properties
classdef Robot < handle
    
    % To do: Add sensor properties, motion model, etc.
    % To do: Abstract sensor to class
    % To do: Abstract motion to class
    properties
        
        ID;                     % Robot's unique ID
        pose;                   % Robot's current position [x; y; theta]
        beliefs;                % Robot's beliefs about the world state
        
        motionController;       % Robot's controller
        motionModel;            % Robot's motion model
        sensors;                % Robot's sensor handles
        
        roles;                  % Software agents
        
        measurements;           % Current measurements in cell form
        odometry;
        last_output;            %
        
    end
    
    methods
        
        % Split into a Copy() method
        function obj = Robot(a)
            
            obj.last_output = zeros(3,1);
            obj.sensors = {};
            
            if nargin == 0
                return;
            end
            
            if ~isa(a, 'Robot')
                display('Invalid argument type');
                return
            end
            
            obj.pose = a.pose;
            obj.ID = a.ID;
            
            obj.motionController = a.motionController.Copy();
            obj.motionModel = a.motionModel.Copy();                  
            for i = 1:numel(a.sensors)
                obj.sensors = [obj.sensors, {a.sensors{i}.Copy()}];
            end
            
        end
        
        % TODO: Make this less hacky somehow
        % Perhaps store a robot handle in the sensors instead of ID?
        function [] = SetID(obj, id)
            
            obj.ID = id;
            if ~isempty(obj.motionController)
                obj.motionController.ownerID = id;
            end
            if ~isempty(obj.motionModel)
                obj.motionModel.ownerID = id;
            end
            if ~isempty(obj.sensors)
                for i = 1:numel(obj.sensors)
                    obj.sensors{i}.ownerID = id;
                end
            end
            
        end
        
        function [] = RegisterMotionController(obj, mc)
            
            obj.motionController = mc;
            mc.ownerID = obj.ID;
            
        end
        
        function [] = RegisterMotionModel(obj, mm)
            
            obj.motionModel = mm;
            mm.ownerID = obj.ID;
            
        end
        
        function [] = RegisterSensor(obj, s)
            
            obj.sensors = [obj.sensors, {s}];
            s.ownerID = obj.ID;
            
        end
        
        % Roles should be registered in lowest-level first order
        function [] = RegisterRole(obj, r)
            
            obj.roles = [obj.roles, r];
            r.ownerID = obj.ID;
            
        end
        
        function [] = Step(obj, state)                       
            
            [idMap, ~] = state.BuildMaps();
            currPose = state.poses(:,idMap.Forward(obj.ID));   
            obj.beliefs = currPose; %TODO Placeholder for localization results
            
            % Generate control outputs and apply motion
            u = obj.motionController.GenerateOutputs(obj.beliefs);
            obj.pose = obj.motionModel.GenerateMotion(obj.pose, u);
            
            prevPose = currPose - obj.last_output;
            prevPose(3) = wrapToPi(prevPose(3));
            obj.last_output = u;
            
            % TODO generalize to sensor and move to module
            obj.odometry = MeasurementRelativePose(currPose, prevPose, zeros(3));
            obj.odometry.covariance = obj.motionModel.covariance;
            obj.odometry.observer_id = obj.ID;
            obj.odometry.target_id = obj.ID;
            obj.odometry.observer_time = state.time;
            obj.odometry.target_time = state.time - 1;
            
            % Don't do any of the below on the first iteration
            % TODO Such a hack..
            if state.time == 0
                obj.odometry = {};                
            end
            
            % TODO Has to come after odometry! Fix this!!
            obj.GenerateMeasurements(state);             
            obj.roles(1).PushMeasurements(obj.measurements);
            
            if state.time == 0
                return
            end
            
            if ~isempty(obj.roles)            
                obj.roles(1).ProcessMeasurements();
            end
            
        end
        
        function [meas] = GetMeasurements(obj)
                        
            meas = obj.measurements;
            
        end
    end
    
    methods(Access = private)
        
        function GenerateMeasurements(obj, state)
            
            meas = {};
            
            for i = 1:numel(obj.sensors);
                meas = [meas; obj.sensors{i}.GenerateMeasurements(state)];
            end
            if ~isempty(obj.odometry)
                meas = [meas; {obj.odometry}];
            end
            
            obj.measurements = meas;
            
        end
        
        % TODO: Convert to sensor module
%         function [measurements] = GetMeasurementsRangeBearing(obj, state)
%             
%             N = numel(state.poses);
%             p = obj.pose;
%             measurements = {};
%             
%             for i = 1:N
%                 
%                 target_id = state.ids(i);
%                 if target_id == obj.ID
%                     continue
%                 end
%                 target_pose = state.poses(i);
%                 rel = target_pose - p;
%                 if norm(rel.position) > obj.sensor_range
%                     continue
%                 end
%                 
%                 m = MeasurementRangeBearing(p, target_pose, obj.sensor_covariance);
%                 m.observer_id = obj.ID;
%                 m.target_id = target_id;
%                 measurements = [measurements, {m}];
%                 
%             end
%             
%         end
        
        function [o] = MoveRandom(obj, w)
            
            bounds = w.dims/2;
            p = obj.pose.position;
            if p(1) > bounds(1)
                p(1) = bounds(1);
            elseif p(1) < -bounds(1)
                p(1) = -bounds(1);
            end
            if p(2) > bounds(2)
                p(2) = bounds(2);
            elseif p(2) < -bounds(2)
                p(2) = -bounds(2);
            end
            obj.pose.position = p;
            
        end
        
    end
    
end
