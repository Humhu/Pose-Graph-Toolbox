% Produces fractal pyramid points with optional noise
%
% Inputs:
%   center - 3 X 1 center of fractal
%   b - Branching factor
%   r - Radius
%   f - Scale factor
%   d - Depth
%   covariance - 3 X 3 noise covariance for multivariate Gaussian noise
%   seed - Random number generator seed
%
% Ouputs:
%   positions - d X 1 cell array. Each element is a cell array of (d x f) 3 X
%   b matrices of positions

function [positions] = GenerateFractal(center, b, r, f, d, covariance, seed)

s = RandStream('mt19937ar', 'Seed', seed);
prevStream = RandStream.getGlobalStream();
RandStream.setGlobalStream(s);

positions = cell(d, 1);
positions{1} = {center};

for i = 2:d        
    
    positions{i} = cell(1, b*numel(positions{i - 1}));
    
    for j = 1:numel(positions{i - 1})
        
        pivot = positions{i - 1}{j};
        
        noise = mvnrnd(zeros(3,1), covariance, b)';
        points = [CirclePoints(zeros(2,1), r*f^(i - 2), b);
                  zeros(1, b)];
        points = points + noise;                
        points = bsxfun(@plus, points, pivot);                
        points = mat2cell(points, 3, ones(1,b));
        
        positions{i}((j - 1)*b + 1 : j*b) = points;
        
    end
    
end

RandStream.setGlobalStream(prevStream);

end

function [points] = CirclePoints(center, radius, n)

theta = linspace(0, 2*pi, n + 1);
theta(end) = [];

points = radius*[cos(theta); sin(theta)];
points = bsxfun(@plus, points, center);

end

