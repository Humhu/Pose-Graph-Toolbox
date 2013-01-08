%process errors

T = numel(err);
N = size(err{1},2);

x_mean = zeros(T,N);
y_mean = zeros(T,N);
a_mean = zeros(T,N);
max_errs = zeros(T,N);

for i = 1:T
    ei = err{i};
    ex = ei(1:3:end,:);
    ey = ei(2:3:end,:);
    ea = ei(3:3:end,:);
    
    xm = mean(ex,1);
    ym = mean(ey,1);
    am = mean(ea,1);
    
    x_mean(i,:) = xm;
    y_mean(i,:) = ym;
    a_mean(i,:) = am;
    
end