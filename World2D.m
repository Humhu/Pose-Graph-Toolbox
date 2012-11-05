% Represents world and visualization
classdef World2D < handle
    
    properties
        dims;           % World size centered at 0
        time;           % World time
        robots = Robot; % Robot objects handles
        num_robots;     % Number of robots
        
        default_sensor_range = 0.5;
        default_sensor_mean = [0;0;0];
        default_sensor_cov = 0.001*eye(3);
        
    end
    
    methods
        
        function obj = World2D(world_size)
            if nargin == 0
                return
            end
            
            obj.dims = reshape(world_size, 2, 1);
            
        end
        
        function InitRobots(obj, N)
            
            dim_scale = reshape(obj.dims, 1, 1, 2);
            dim_offset = dim_scale/2;
            positions = bsxfun(@times, dim_scale, rand(N,1,2));
            positions = bsxfun(@minus, positions, dim_offset);
            orientations = 2*pi*rand(N,1);
            
            obj.robots(N,1) = Robot;
            
            for i = 1:N
                r = obj.robots(i);
                r.pose = Pose2D(positions(i,:,:),orientations(i));
                r.id = i;
                r.sensor_range = obj.default_sensor_range;
                r.sensor_mean = obj.default_sensor_mean;
                r.sensor_covariance = obj.default_sensor_cov;
                r.sensor_type = 'RelativePose';               
            end
            
            obj.num_robots = N;
            
        end
        
        function [relations, ids] = GetRelations(obj)
            
            relations = [];
            ids = [];
            
            for i = 1:obj.num_robots;
                
                [rel_i, id_i] = obj.robots(i).GetRelations(obj);
                relations = [relations, rel_i];
                ids = [ids, id_i];
                
            end
            
        end
        
        function [measurements] = GetMeasurements(obj)
            
            measurements = [];
            
            for i = 1:obj.num_robots;
                
                [rel_i] = obj.robots(i).GetMeasurements(obj);
                if isempty(rel_i)
                    continue
                end
                measurements = [measurements, rel_i];
                
            end
            
        end
        
        function [poses] = GetPoses(obj)
           
            poses = reshape([obj.robots.pose], size(obj.robots));
            
        end
        
    end
    
end