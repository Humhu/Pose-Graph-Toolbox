function[] = DrawLine(varargin)

s = varargin{1};
e = varargin{2};
if nargin > 2
    linespec = varargin(3:end);
else
    linespec = {};
end

x = [s(1); e(1)];
y = [s(2); e(2)];
z = [s(3); e(3)];
plot3(x,y,z,linespec{:});