% Represents world and visualization
classdef World2D < handle
    
    properties
        dims;           % World size centered at 0
        time;           % World time
        robots;         % Robot objects handles
        num_robots;     % Number of robots
        recorder;
        
        default_sensor_range = Inf;
        default_sensor_mean = [0;0;0];
        default_sensor_cov = 0.001*eye(3);
        default_sensor_type = 'RelativePose'
        
    end
    
    methods
        
        function obj = World2D(a)
            if nargin == 0
                return
            end
            
            if isa(a, 'double')
                obj.dims = reshape(a, 2, 1);
                return
            end
            
            if ~isa(a, 'World2D')
                display('Invalid argument type');
                return
            end
            
            obj.dims = a.dims;
            obj.time = a.time;
            obj.num_robots = a.num_robots;
            obj.default_sensor_range = a.default_sensor_range;
            obj.default_sensor_mean = a.default_sensor_mean;
            obj.default_sensor_cov = a.default_sensor_cov;
            obj.robots = Robot;
            obj.robots(obj.num_robots,1) = Robot;
            
            for i = 1:obj.num_robots
               obj.robots(i) = Robot(a.robots(i)); 
            end
            
        end
        
        function InitRobots(obj, N)
            
            dim_scale = reshape(obj.dims/2, 1, 1, 2);            
            positions = bsxfun(@times, dim_scale, 2*rand(N,1,2) - 1);            
            orientations = 2*pi*rand(N,1);
            
            obj.robots = Robot;
            obj.robots(N,1) = Robot;
            
            for i = 1:N
                r = obj.robots(i);
                r.pose = Pose2D(positions(i,:,:),orientations(i));
                r.id = i;
                r.sensor_range = obj.default_sensor_range;
                r.sensor_mean = obj.default_sensor_mean;
                r.sensor_covariance = obj.default_sensor_cov;
                r.sensor_type = obj.default_sensor_type;
            end
            
            obj.num_robots = N;
            obj.recorder = Recorder2D(N);
            obj.recorder.Append(obj.GetPoses);
            
        end
        
        function [] = TickTime(obj)
           
            for i = 1:obj.num_robots
               
                r = obj.robots(i);
                r.ExecuteMovement(obj);
                
            end
            
            obj.recorder.Append(obj.GetPoses);
            
        end
        
        function [measurements] = GetMeasurements(obj)
            
            measurements = {};
            
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
       
        function [err] = GetErrors(obj, w)
            
            if ~isa(w, 'World2D')
                display('Invalid argument type')
            end
            
            err = double([obj.robots.pose] - [w.robots.pose]);
        end
        
    end
    
end