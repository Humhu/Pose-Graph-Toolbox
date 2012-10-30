% Test mean of cos(a + t) where a is Gaussian noise and t is constant

sample_size = 1000;
num_bins = 20;
bin_centers = linspace(-5,5,num_bins);

% Produce a
a = normrnd(0, 1, sample_size, 1);
a_mean = mean(a);
hist(a, bin_centers);
title(['Mean: ', num2str(a_mean)]);

t_datapoints = 100;
t = linspace(-pi, pi, t_datapoints)';
num_trials = length(t);

means = zeros(num_trials, 1);
f = @(a, t) cos(a + t);

for i = 1:num_trials
    
    t_in = t(i)*ones(sample_size, 1);
    d = f(a, t_in);
    means(i) = mean(d);
    
end

figure;
hold on;
plot(t, means, 'b-');
plot(t, f(zeros(t_datapoints,1), t), 'r-');
legend('Mean of cos(a + t)', 'cos(t)', 'location', 'best');
xlabel('t');
