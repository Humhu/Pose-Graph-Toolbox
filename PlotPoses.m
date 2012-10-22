% Poses is an N by T matrix, where N is the number of robots and T is the
% number of time steps. poses(n, t) corresponds to robot n's position at
% time step t.
function [] = PlotPoses(poses)

figure;
hold on;

tick_length = 0.025;
robot_size = 0.025;

N = size(poses,1);
T = size(poses,2);

colors = hsv(N);

for i = 1:N
    for j = 1:T
        
        p = poses(i,j);
        x = p.position(1);
        y = p.position(2);
        color = colors(i,:);
        
        % Plot circle
        viscircles([x, y], robot_size, 'linewidth', 0.4, ...
            'edgecolor', color);
        
        % Plot orientation tick
        dx = tick_length*cos(double(p.orientation));
        dy = tick_length*sin(double(p.orientation));
        line_x = [x, x + dx];
        line_y = [y, y + dy];
        line(line_x, line_y, 'linewidth', 0.4, 'color', color);
        
        % Label robot
        text(x, y, num2str(i));
        
    end
end

axis equal