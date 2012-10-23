% Represents world and visualization
classdef World2D < handle
    
    properties
        dims;           % World size
        time;           % World time
        robot_poses;    % Robot poses
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
            axis(obj.axe, [0, obj.dims(1), 0, obj.dims(2)]);
        end
        
        function delete(obj)
            
            close(obj.fig);
            
        end
        
        function InitRobots(obj, N)
            dim_scale = reshape(obj.dims, 1, 1, 2);
            positions = bsxfun(@times, dim_scale, rand(N,1,2));
            orientations = 2*pi*rand(N,1);
            obj.robot_poses = Pose2D(positions, orientations);
            obj.num_robots = N;
            obj.colors = hsv(N);
            
        end
        
        function [relations] = GetRelations(obj)
            
            relations(obj.num_robots, obj.num_robots) = Pose2D;
            
            for i = 1:obj.num_robots
                for j = 1:obj.num_robots
                    if i == j
                        relations(i,j) = Pose2D(reshape([0,0],1,1,2), 0);
                        continue
                    end
                    pi = obj.robot_poses(i);
                    pj = obj.robot_poses(j);
                    rel_pos = pj.position - pi.position;
                    rel_bearing = Orientation1D(atan2(rel_pos(2), rel_pos(1))) - pi.orientation;
                    relations(i,j) = Pose2D(reshape(rel_pos, 1, 1, 2), rel_bearing);
                end
            end
            
        end
        
        function [ranges, angles] = GetMeasurements(obj, covariance)
            
            relations = obj.GetRelations;
            ranges = zeros(obj.num_robots, obj.num_robots);
            angles(obj.num_robots, obj.num_robots) = Orientation1D;
            
            for i = 1:obj.num_robots
                for j = 1:obj.num_robots
                    noise = mvnrnd([0; 0], covariance);
                    ranges(i,j) = norm(relations(i,j).position) + noise(1);
                    angles(i,j) = relations(i,j).orientation + Orientation1D(noise(2));
                end
            end
            
            for i = 1:obj.num_robots
                ranges(i,i) = 0;
                angles(i,i) = Orientation1D(0);
            end
            
        end
        
        function PlotPoses(obj)
            
            cla(obj.axe);
            
            for i = 1:obj.num_robots
                
                p = obj.robot_poses(i);
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
        
        function PlotMeasurements(obj, ranges, angles)
            
            for i = 1:obj.num_robots
                
                color = obj.colors(i,:);
                
                for j = 1:obj.num_robots
                    
                    if(i == j)
                        continue;
                    end
                    
                    p = obj.robot_poses(i);
                    x = p.position(1);
                    y = p.position(2);
                    t = p.orientation;

                    r = ranges(i,j);
                    a = angles(i,j);
                    
                    % Plot orientation tick
                    dx = r*cos(double(t + a));
                    dy = r*sin(double(t + a));
                    line_x = [x, x + dx];
                    line_y = [y, y + dy];
                    line(line_x, line_y, 'linewidth', obj.measurement_thickness, 'color', color);
                    
                end
            end
            
        end
        
    end
    
end