% Represents world and visualization
classdef World2D < handle
    
    properties
        dims;           % World size centered at 0
        time;           % World time
        robots = Robot; % Robot poses
        num_robots;     % Number of robots
        
        fig;            % Visualization figure handle
        axe;            % Visualization axes handle
        colors;         % Visualization colors
        tick_length     = 0.025;
        tick_thickness  = 2;
        robot_size      = 0.025;
        robot_thickness = 2;
        measurement_thickness = 0.4;
        
    end
    
    methods
        
        function obj = World2D(world_size)
            if nargin == 0
                return
            end
            
            obj.dims = reshape(world_size, 2, 1);
            obj.fig = figure;
            obj.axe = axes;
            axis(obj.axe, 'equal');
            axis(obj.axe, [-obj.dims(1)/2, obj.dims(1)/2, ...
                -obj.dims(2)/2, obj.dims(2)/2]);
        end
        
        function delete(obj)
            
            close(obj.fig);
            
        end
        
        function InitRobots(obj, N)
            
            dim_scale = reshape(obj.dims, 1, 1, 2);
            dim_offset = dim_scale/2;
            positions = bsxfun(@times, dim_scale, rand(N,1,2));
            positions = bsxfun(@minus, positions, dim_offset);
            orientations = 2*pi*rand(N,1);
            
            obj.robots(N) = Robot;
            
            for i = 1:N
                r = obj.robots(i);
                r.pose = Pose2D(positions(i,:,:),orientations(i));
                r.id = i;
                r.sensor_range = Inf;% 0.5;
                r.sensor_mean = [0;0];
                r.sensor_covariance = 0.01*eye(2);
            end
            
            obj.num_robots = N;
            obj.colors = hsv(N);
            
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
        
        function [measurements, ids] = GetMeasurements(obj)
            
            measurements = [];
            ids = [];
            
            for i = 1:obj.num_robots;
                
                [rel_i, id_i] = obj.robots(i).GetMeasurements(obj);
                measurements = [measurements, rel_i];
                ids = [ids, id_i];
                
            end
            
        end
        
        function PlotPoses(obj)
            
            cla(obj.axe);
            
            for i = 1:obj.num_robots
                
                p = obj.robots(i).pose;
                x = p.position(1);
                y = p.position(2);
                color = obj.colors(i,:);
                
                % Plot circle
                viscircles([x, y], obj.robot_size, 'linewidth', obj.robot_thickness, ...
                    'edgecolor', color);
                
                % Plot orientation tick
                dx = obj.tick_length*cos(double(p.orientation));
                dy = obj.tick_length*sin(double(p.orientation));
                line_x = [x, x + dx];
                line_y = [y, y + dy];
                line(line_x, line_y, 'linewidth', obj.tick_thickness, 'color', color);
                
                % Label robot
                text(x, y, num2str(i));
                
            end
            
        end
        
        function PlotMeasurements(obj, measurements, ids)
            
            num_meas = size(ids, 2);
            
            for i = 1:num_meas
                
                m = measurements(i);
                id1 = ids(1,i);
                
                color = obj.colors(id1,:);
                                
                p = obj.robots(id1).pose;
                x = p.position(1);
                y = p.position(2);
                t = p.orientation;
                
                r = m.range
                a = m.bearing
                
                % Plot orientation tick
                dx = r*cos(double(t + a))
                dy = r*sin(double(t + a))
                line_x = [x, x + dx];
                line_y = [y, y + dy];
                line(line_x, line_y, 'linewidth', obj.measurement_thickness, 'color', color);
                
            end
            
        end
        
    end
    
end