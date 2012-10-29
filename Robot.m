% Represents a robot and its properties
classdef Robot < handle
    
    % To do: Add sensor properties, motion model, etc.
    properties
        
        pose;
        id;
        
        sensor_range;
        sensor_mean;
        sensor_covariance;
        
    end
    
    methods
        
        function obj = Robot()
            if nargin == 0
                return
            end
            
        end
        
        function e = eq(A,B)
            
            e = A.id == B.id;
            
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
                
                rel_bearing = Orientation1D(atan2(rel_pos(2), rel_pos(1))) - p.orientation;
                relations = [relations, Pose2D(reshape(rel_pos, 1, 1, 2), rel_bearing)];
                ids = [ids, [obj.id; ri.id]];
                
            end
            
        end
        
        function [measurements, ids] = GetMeasurements(obj, world)
            
            [relations, rel_ids] = obj.GetRelations(world);
            num_neighbors = size(rel_ids,2);
            
            measurements = MeasurementRangeBearing;
            ids = [];
            
            j = 1;
            for i = 1:num_neighbors
                
                if(norm(relations(i).position) > obj.sensor_range)
                    continue
                end
                
                noise = mvnrnd(obj.sensor_mean, obj.sensor_covariance);
                range = norm(relations(i).position) + noise(1);
                if range < 0
                    range = 0;
                end
                angle = relations(i).orientation + Orientation1D(noise(2));                
                measurements(j) = MeasurementRangeBearing(range, angle, ...
                    obj.sensor_covariance);
                ids(:,j) = rel_ids(:,i);
                j = j + 1;
                
            end
            
        end
        
    end
    
end