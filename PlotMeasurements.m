function [] = PlotMeasurements(poses, ranges, angles)

PlotPoses(poses);

N = numel(poses);

for i = 1:N
    
    for j = 1:N
        
        if(i == j) 
            continue;
        end
        
        p = poses(i);
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
        line(line_x, line_y, 'linewidth', 0.4, 'color', [1 0 0]);               
        
    end
end

axis equal