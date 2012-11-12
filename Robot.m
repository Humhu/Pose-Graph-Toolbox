% Represents a robot and its properties
classdef Robot < handle
    
    % To do: Add sensor properties, motion model, etc.
    % To do: Add controller object to generate movement
    properties
        
        pose;
        id;
        
        sensor_type;
        sensor_range;
        sensor_mean;
        sensor_covariance;
        
        motion_mode = 'Orbit';  % Motion generation
        motion_mag = 0.1;
        motion_mean = zeros(3,1);
        motion_covariance = zeros(3);%0.1*eye(3);
        
    end
    
    methods
        
        function obj = Robot(a)
            if nargin == 0
                return
            end
            
            if ~isa(a, 'Robot')
                display('Invalid argument type');
                return
            end
            
            obj.pose = a.pose;
            obj.id = a.id;
            obj.sensor_type = a.sensor_type;
            obj.sensor_range = a.sensor_range;
            obj.sensor_mean = a.sensor_mean;
            obj.sensor_covariance = a.sensor_covariance;
            
        end
        
        function [] = ExecuteMovement(obj, w)
           
            if strcmp(obj.motion_mode, 'Random')
                obj.MoveRandom(w);
            elseif strcmp(obj.motion_mode, 'Orbit')
                obj.MoveOrbit(w);
            end
            
        end
        
        function [measurements] = GetMeasurements(obj, world)
            
            if strcmp(obj.sensor_type, 'RangeBearing')
                measurements = obj.GetMeasurementsRangeBearing(world);
            elseif strcmp(obj.sensor_type, 'RelativePose')
                measurements = obj.GetMeasurementsRelativePose(world);
            end                        
            
        end
    end
    
    methods(Access = private)
                
        function [measurements] = GetMeasurementsRangeBearing(obj, world)
            
            N = numel(world.robots);
            p = obj.pose;                        
            measurements = {};
            
            for i = 1:num_neighbors
                
                r = world.robots(i);
                if r == obj
                    continue
                end
                
                rel = r.pose - p;
                if norm(rel.position) > obj.sensor_range
                    continue
                end                              
                
                m = MeasurementRangeBearing(p, r.pose, obj.sensor_covariance);
                m.observer_id = obj.id;
                m.target_id = r.id;
                measurements = [measurements, {m}];
                
            end
            
        end
        
        function [measurements] = GetMeasurementsRelativePose(obj, world)
           
            N = numel(world.robots);
            p = obj.pose;
            measurements = {};
            
            for i = 1:N
               
                r = world.robots(i);
                if r == obj
                    continue
                end
                
                rel = r.pose - p;
                if norm(rel.position) > obj.sensor_range
                    continue
                end
                
                m = MeasurementRelativePose(p, r.pose, obj.sensor_covariance);
                m.observer_id = obj.id;
                m.target_id = r.id;
                measurements = [measurements, {m}];
                
            end
            
        end        
        
        function [o] = MoveRandom(obj, w)
            
            o = zeros(3,1);
            d = mvnrnd(obj.motion_mean, obj.motion_covariance);
            d = obj.motion_mag*(d./norm(d));
            obj.pose = obj.pose + reshape(d,1,1,3);
            
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
       
        function [o] = MoveOrbit(obj, w)
            
            n = mvnrnd(obj.motion_mean, obj.motion_covariance);
            r = norm(obj.pose.position);
            da = obj.motion_mag/r;
            R = [cos(da), -sin(da);
                sin(da), cos(da)];
            prev_pos = obj.pose.position;
            obj.pose.position = R*obj.pose.position + n(1:2)';
            obj.pose.orientation = obj.pose.orientation + da + n(3);
            o = [prev_pos - obj.pose.position; da];
            
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
