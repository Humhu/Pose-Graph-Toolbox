% Represents a robot and its properties
classdef Robot < handle
    
    % To do: Add sensor properties, motion model, etc.
    properties
        
        pose;
        id;
        
        sensor_type;
        sensor_range;
        sensor_mean;
        sensor_covariance;
        
        motion_mode = 'Random';  % Motion generation
        motion_mag = 0.1;
        motion_mean = zeros(3,1);
        motion_covariance = 0.1*eye(3);
        
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
        
        function [relations, ids] = GetRelations(obj, world)
            
            relations = [];
            ids = [];
            p = obj.pose;
            
            for i = 1:world.num_robots
                ri = world.robots(i);
                if ri == obj
                    continue
                end
                
                rp = ri.pose;
                rel_pos = rp.position - p.position;
                
                rel_ori = rp.orientation - p.orientation;
                relations = [relations, Pose2D(reshape(rel_pos, 1, 1, 2), rel_ori)];
                ids = [ids, [obj.id; ri.id]];
                
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
        
        % TODO: Fix growing in loop by preallocating and trimming?
        function [measurements] = GetMeasurementsRangeBearing(obj, world)
            
            [relations, rel_ids] = obj.GetRelations(world);
            num_neighbors = size(rel_ids,2);
            
            measurements = {};
            j = 1;
            for i = 1:num_neighbors
                
                rel = relations(i);
                r = norm(rel.position);
                if(r > obj.sensor_range)
                    continue
                end
                
                noise = mvnrnd(obj.sensor_mean, obj.sensor_covariance);
                r = r + noise(1);
                if r < 0
                    r = 0;
                end
                
                b = Orientation1D(atan2(rel.position(2), rel.position(1))) - obj.pose.orientation;
                b = b + Orientation1D(noise(2));
                
                measurements{j} = MeasurementRangeBearing(r, b, ...
                    obj.sensor_covariance, rel_ids(1,i), rel_ids(2,i));
                j = j + 1;
                
            end
            
        end
        
        function [measurements] = GetMeasurementsRelativePose(obj, world)
            
            [relations, rel_ids] = obj.GetRelations(world);
            num_neighbors = size(rel_ids, 2);
            
            measurements = {};
            j = 1;
            for i = 1:num_neighbors
               
                pos = relations(i).position;
                
                if(norm(pos) > obj.sensor_range)
                    continue
                end
                
                ori = relations(i).orientation;
                noise = mvnrnd(obj.sensor_mean, obj.sensor_covariance)';
                t = double(obj.pose.orientation);
                R = [cos(t), sin(t);
                     -sin(t), cos(t)];
                disp = R*pos + noise(1:2,1);
                rel_ori = ori + noise(3);
                measurements{j} = MeasurementRelativePose(disp, rel_ori, ...
                    obj.sensor_covariance, rel_ids(1,i), rel_ids(2,i));
                j = j + 1;
                
            end
            
        end
        
    end
    
    
end
