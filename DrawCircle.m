function [] = DrawCircle(varargin)

center = varargin{1};
radius = varargin{2};
n = varargin{3};

if nargin > 3
    linespec = varargin(4:end);
else
    linespec = {};
end

t = linspace(0, 2*pi, n+1);
x = center(1) + radius*cos(t);
y = center(2) + radius*sin(t);
z = center(3)*ones(length(t),1);
plot3(x,y,z,linespec{:});
