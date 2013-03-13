% Factorial testing over n, alpha with fixed random seeds

% Use same seeds for each trial per experiment
seeds = 1:1;
num_trials = numel(seeds);
trial_length = 40; % Number of steps to simulate per trial

% Test parameters
%test_time_scales = [1, 2, 3];
test_time_scales = 3;
%test_time_overlaps = [0, 1, 2];
test_time_overlaps = 0;

[time_scale_v, time_overlap_v] = ndgrid(test_time_scales, test_time_overlaps);
num_experiments = numel(time_scale_v);

estimate_errors = zeros(num_experiments, 4, trial_length);
baseline_errors = zeros(num_experiments, 4, trial_length);
odo_errors = zeros(num_experiments, 4, trial_length);
baseline_ratios = zeros(num_experiments, 4, trial_length);
odo_ratios = zeros(num_experiments, 4, trial_length);

% Other fixed parameters
world_dims = [80;20];
%chain_holdoff = [1; 1; 1];
chain_holdoff = [0; 0; 0];
representative_holdoff = [0; 0; 0];

% Fractal positioning parameters
center = [-30; 0; 0]; %zeros(3,1);
d = 3; % Hierarchy depth
b = 3; % Branching factor
f = 0.5; % Fractal size
r = 5; % Starting separation
covariance = zeros(3);

% Timing
exp_mod = 10;
tri_mod = 5;

%% Experiment loops
tic();
for k = 1:num_experiments
    
    if mod(k, exp_mod) == 1
        now = toc();
        runtime_estimate = toc()/(k-1)*num_experiments;
        fprintf('Starting experiment %d/%d. ETA: %f sec\n', k, ...
            num_experiments, runtime_estimate - toc());
    end
    
    % Store errors
    experiment_estimate_errors = zeros(num_trials, 4, trial_length);
    experiment_baseline_errors = zeros(num_trials, 4, trial_length);
    experiment_odo_errors = zeros(num_trials, 4, trial_length);
    experiment_baseline_ratios = zeros(num_trials, 4, trial_length);
    experiment_odo_ratios = zeros(num_trials, 4, trial_length);
    
    tscale = time_scale_v(k);
    toverlap = time_overlap_v(k);
    
    time_scales = tscale.^(2:-1:0);
    time_overlaps = [1,toverlap,0];
    
    % Initialize pre-defined template robot
    r_template = GenerateStraightRobot(world_dims);
    
    % Run trials for this experimental configuration
    for s = 1:num_trials
        
        if mod(s, tri_mod) == 1
            now = toc();
            done = (k-1)*num_trials + s-1;
            to_do = num_trials*num_experiments;
            runtime_est = now/done*to_do;
            fprintf('\tTrial %d/%d. ETA: %f\n', s, num_trials, runtime_est - now);
        end
        
        % Seed random number generator
        seed = seeds(s);
        stream = RandStream('mt19937ar', 'Seed', seed);
        RandStream.setGlobalStream(stream);
        
        % Start simulation
        sim = Simulator2D(world_dims, false);
        
        % Generate robots
        positions = GenerateFractal(center, b, r, f, d, covariance, 0);
        positions = positions{end};
        positions = [positions{:}];
        positions(3,:) = wrapToPi(positions(3,:));
        robots = InitRobots(r_template, {positions});
        grouping = GenerateGrouping(d, b);

        sim.AddRobots(robots);
        
        AssignGrouping(robots, grouping, time_scales, time_overlaps, ...
            chain_holdoff, representative_holdoff); % Set up hierarchy and assign roles
        
        % Add robots to simulator
        sim.Initialize();
        
        gn = GNSolver(1E-6, 100);
        init = gn.Solve(sim.world.state);
        
        root = sim.world.robots(1).roles(1); %hardcoded for now
        root.Initialize(init); % Initializes the entire tree
        
        leafs = ChainedGraph.empty(0, numel(sim.world.robots));
        for i = 1:numel(sim.world.robots)
            leafs(i) = sim.world.robots(i).roles(end).chained_graph;
        end
        
        % Run simulator and calculate errors
        r_template = sim.world.robots(1);
        r1 = sim.world.robots(2);
        r2 = sim.world.robots(3);
        r3 = sim.world.robots(4);
        
        trial_estimate_errors = zeros(d+1, trial_length);
        trial_baseline_errors = zeros(d+1, trial_length);
        trial_odo_errors = zeros(d+1, trial_length);
        
        cg0_0 = sim.world.robots(1).roles(1).chained_graph;
        cg1_0 = sim.world.robots(1).roles(2).chained_graph;
        cg2_0 = sim.world.robots(1).roles(3).chained_graph;
        
        for i = 1:trial_length                                                
            
            truth = sim.history(1:sim.history_ind-1);
            [cg_errs, baseline_errs, odo_errs] = CalculateLeafGraphErrors(leafs, truth);                        
            trial_estimate_errors(:, i) = mean(cg_errs, 2);
            trial_baseline_errors(:, i) = mean(baseline_errs, 2);
            trial_odo_errors(:, i) = mean(odo_errs, 2);
            
            eg = trial_estimate_errors(4,i);
            e0 = trial_estimate_errors(3,i);
            e1 = trial_estimate_errors(2,i);
            e2 = trial_estimate_errors(1,i);
            fprintf('\teg: %f e0: %f e1: %f e2: %f\n', eg, e0, e1, e2);

            fprintf(['Step: ', num2str(i), '\n']);
            
            sim.Step();
            
            n0 = numel(cg0_0.subgraph(end).measurements);
            n1 = numel(cg1_0.subgraph(end).measurements);
            n2 = numel(cg2_0.subgraph(end).measurements);
            fprintf(['\tCG0 T: ', num2str(cg0_0.time_scope), '\tbT: ', ...
                num2str(cg0_0.base_time), '\t|m|: ', num2str(n0), '\n']);
            fprintf(['\tCG1 T: ', num2str(cg1_0.time_scope), '\tbT: ', ...
                num2str(cg1_0.base_time), '\t|m|: ', num2str(n1), '\n']);
            fprintf(['\tCG2 T: ', num2str(cg2_0.time_scope), '\tbT: ', ...
                num2str(cg2_0.base_time), '\t|m|: ', num2str(n2), '\n']);                                    
            
        end
        trial_baseline_ratios = trial_estimate_errors./trial_baseline_errors;
        trial_odo_ratios = trial_estimate_errors./trial_odo_errors;
        
        experiment_estimate_errors(s,:,:) = trial_estimate_errors;
        experiment_baseline_errors(s,:,:) = trial_baseline_errors;
        experiment_odo_errors(s,:,:) = trial_odo_errors;
        experiment_baseline_ratios(s,:,:) = trial_baseline_ratios;
        experiment_odo_ratios(s,:,:) = trial_odo_ratios;
        
    end
    
    estimate_errors(k,:,:) = mean(experiment_estimate_errors, 1);
    baseline_errors(k,:,:) = mean(experiment_baseline_errors, 1);
    odo_errors(k,:,:) = mean(experiment_odo_errors, 1);
    baseline_ratios(k,:,:) = mean(experiment_baseline_ratios, 1);
    odo_ratios(k,:,:) = mean(experiment_odo_ratios, 1);
    
end

%% Plotting
figure;
hold on;
plot(0:trial_length-1, squeeze(baseline_ratios(1,1,:)), 'ro-');
plot(0:trial_length-1, squeeze(baseline_ratios(1,2,:)), 'bx-');
plot(0:trial_length-1, squeeze(baseline_ratios(1,3,:)), 'g+-');
%plot(0:trial_length-1, squeeze(baseline_ratios(1,4,:)), 'm^-');
plot([0,trial_length-1], [1,1], 'k--');
%plot(0:n_step-1, est_baseline_ratio(1,:), 'r');
xlabel('Step number');
ylabel('CG error/GN error');
legend('k = 2', 'k = 1', 'k = 0', 'location', 'best');
title('Baseline Performance Ratio vs. Steps');

figure;
hold on;
plot(0:trial_length-1, squeeze(odo_ratios(1,1,:)), 'ro-');
plot(0:trial_length-1, squeeze(odo_ratios(1,2,:)), 'bx-');
plot(0:trial_length-1, squeeze(odo_ratios(1,3,:)), 'g+-');
%plot(0:trial_length-1, squeeze(odo_ratios(1,4,:)), 'm^-');
plot([0,trial_length-1], [1,1], 'k--');
%plot(0:n_step-1, est_baseline_ratio(1,:), 'r');
xlabel('Step number');
ylabel('CG error/GN error');
legend('k = 2', 'k = 1', 'k = 0', 'location', 'best');
title('Odometry Performance Ratio vs. Steps');

figure;
hold on;
plot(0:trial_length-1, squeeze(estimate_errors(1,1,:)), 'rx-');
plot(0:trial_length-1, squeeze(baseline_errors(1,1,:)), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Depth 2 Estimate vs. Baseline Localized Error');

figure;
hold on;
plot(0:trial_length-1, squeeze(estimate_errors(1,2,:)), 'rx-');
plot(0:trial_length-1, squeeze(baseline_errors(1,2,:)), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Depth 1 Estimate vs. Baseline Localized Error');

figure;
hold on;
plot(0:trial_length-1, squeeze(estimate_errors(1,3,:)), 'rx-');
plot(0:trial_length-1, squeeze(baseline_errors(1,3,:)), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Depth 0 Estimate vs. Baseline Localized Error');

figure;
hold on;
plot(0:trial_length-1, squeeze(estimate_errors(1,4,:)), 'rx-');
plot(0:trial_length-1, squeeze(baseline_errors(1,4,:)), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Global Frame Estimate vs. Baseline Localized Error');

figure;
hold on;
plot(0:trial_length-1, squeeze(estimate_errors(1,1,:)), 'rx-');
plot(0:trial_length-1, squeeze(baseline_errors(1,1,:)), 'r.-');
plot(0:trial_length-1, squeeze(estimate_errors(1,2,:)), 'gx-');
plot(0:trial_length-1, squeeze(baseline_errors(1,2,:)), 'g.-');
plot(0:trial_length-1, squeeze(estimate_errors(1,3,:)), 'bx-');
plot(0:trial_length-1, squeeze(baseline_errors(1,3,:)), 'b.-');
plot(0:trial_length-1, squeeze(estimate_errors(1,4,:)), 'mx-');
plot(0:trial_length-1, squeeze(baseline_errors(1,4,:)), 'm.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('k = 2 CG', 'k = 2 MLE', 'k = 1 CG', 'k = 1 MLE', ...
    'k = 0 CG', 'k = 0 MLE', 'Global CG', 'Global MLE', 'location', 'best')
title('Chained Graph vs. Baseline Localized Error');

%% Plot subsampled trajectories

truth = sim.history(1:sim.history_ind-1);
subtimes = 0:8:sim.history_ind-2;
subtruth = truth.GetSubset([], subtimes);

subplotter = SequencePlotter(world_dims);
subplotter.z_scale = 1;
subplotter.SetColors(numel(sim.world.robots));
subplotter.PlotSequence(subtruth);

%% Get comparison estimate

if false
    gn = GNSolver(1E-6, 100);
    baseline_est = gn.Solve(truth);
    odo = OdometrySolver();
    odo_est = odo.Solve(truth);
    
    N = numel(sim.world.robots);
    
    %Plot results
    truth_plotter = SequencePlotter(world_dims);
    truth_plotter.z_scale = 1;
    truth_plotter.colors = repmat([0,1,0],N,1);
    truth_plotter.PlotSequence(truth);
    
    baseline_plotter = SequencePlotter(world_dims);
    baseline_plotter.z_scale = 1;
    baseline_plotter.Link(truth_plotter);
    baseline_plotter.colors = repmat([0,0,1],N,1);
    baseline_plotter.PlotSequence(baseline_est);
    
    odo_plotter = SequencePlotter(world_dims);
    odo_plotter.z_scale = 1;
    odo_plotter.Link(truth_plotter);
    odo_plotter.colors = repmat([1,0,1],N,1);
    odo_plotter.PlotSequence(odo_est);
    
    belief_plotter = HierarchyPlotter(world_dims);
    belief_plotter.z_scale = 1;
    belief_plotter.Link(truth_plotter);
    belief_plotter.colors = repmat([1,0,0],N,1);
    belief_plotter.PlotBeliefs(sim.world.robots(1).roles(1));
    
    truth_plotter.HideLines();
    truth_plotter.HideLabels();
    baseline_plotter.HideLines();
    baseline_plotter.HideLabels();
    odo_plotter.HideLines();
    odo_plotter.HideLabels();
end
