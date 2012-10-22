function [errors] = CalculateError(poses, ranges, angles)

N = size(poses,1);
errors = zeros(N,N,2);

for i = 1:N
    for j = 1:N
        
        pi = poses(i);
        pix = pi.position(1);
        piy = pi.position(2);
        pit = pi.orientation;
        pj = poses(j);
        pjx = pj.position(1);
        pjy = pj.position(2);
        
        r = ranges(i,j);
        a = angles(i,j);
        
        ex = pjx - pix - r*cos(double(a + pit));
        ey = pjy - piy - r*sin(double(a + pit));
        
        errors(i,j,1) = ex;
        errors(i,j,2) = ey;
        
    end
end
    