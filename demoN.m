clear all;

% Test parameters
num_trials = 100;
cov = [0.0, 0.5];
cov_step = 0.05;
N = [5,100];
N_step = 5;
world_size = [1;1];

cov_trials = cov(1):cov_step:cov(2);
N_trials = N(1):N_step:N(2);

num_covs = length(cov_trials);
num_N = length(N_trials);

avg_err_results = zeros(num_N, num_covs, num_trials);
max_err_results = zeros(num_N, num_covs, num_trials);
timing_results = zeros(num_N, num_covs, num_trials);

fprintf(['Beginning trials...\n']);   
total_trials = num_N*num_covs*num_trials;    

for i = 1:num_N
    trial_num = (i-1)*num_covs*num_trials;
    fprintf(['\tTrial ', num2str(trial_num),'/', num2str(total_trials),'\n']);
    N_val = N_trials(i);
    
    for j = 1:num_covs
        cov_val = cov_trials(j)*eye(2);
        
        for k = 1:num_trials
            
            pos = InitRobots(N_val, world_size);            
            meas = GetMeasurements(pos, cov_val);
            tic;
            est = bsxfun(@plus, pos(:,1), EstimatePositions(pos, meas));
            
            err = sqrt(sum((pos - est).^2,1));
            avg_err_results(i,j,k) = mean(err);
            max_err_results(i,j,k) = max(err);        
            timing_results(i,j,k) = toc;
            
        end
    end
end

%% Process results and plot

avg_err_mean = mean(avg_err_results, 3);
avg_err_cov = var(avg_err_results, 0, 3);

figure;
surf(cov_trials, N_trials, avg_err_mean);
xlabel('variance');
ylabel('N');
zlabel('Mean error average');
title('Mean error averages over 100 trials per parameter');

figure;
surf(cov_trials, N_trials, avg_err_cov);
xlabel('variance');
ylabel('N');
zlabel('Mean error variance');
title('Mean error variances over 100 trials per parameter');

max_err_max = max(max_err_results, [], 3);
max_err_cov = var(max_err_results, 0, 3);

figure;
surf(cov_trials, N_trials, max_err_max);
xlabel('variance');
ylabel('N');
zlabel('Max error max');
title('Max error max over 100 trials per parameter');

figure;
surf(cov_trials, N_trials, max_err_cov);
xlabel('variance');
ylabel('N');
zlabel('Max error variance');
title('Max error variances over 100 trials per parameter');

timing_mean = mean(timing_results, 3);
timing_cov = var(timing_results, 0, 3);

figure;
surf(cov_trials, N_trials, timing_mean);
xlabel('variance');
ylabel('N');
zlabel('Mean runtime');
title('Mean runtime over 100 trials per parameter');

figure;
surf(cov_trials, N_trials, timing_cov);
xlabel('variance');
ylabel('N');
zlabel('Runtime variance');
title('Runtime variance max over 100 trials per parameter');

