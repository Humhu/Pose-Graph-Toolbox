% Factorial testing over n, alpha with fixed random seeds

% Use same seeds for each trial per experiment
seeds = 1:1;
num_trials = numel(seeds);
trial_length = 400; % Number of steps to simulate per trial

% Test parameters
time_scales = [40,8,1];
time_overlaps = [4,1,0];

% Other fixed parameters
world_dims = [80;80];
%chain_holdoff = [1; 1; 1];
chain_holdoff = [0; 0; 0];
representative_holdoff = [6; 0; 0];

% Fractal positioning parameters
center = [5; 0; 0]; %zeros(3,1);
d = 3; % Hierarchy depth
b = 3; % Branching factor
f = 0.5; % Fractal size
r = 5; % Starting separation
covariance = zeros(3);

trial_latest_cge = zeros(d+1, trial_length, num_trials);
trial_latest_be = zeros(d+1, trial_length, num_trials);
trial_latest_oe = zeros(d+1, trial_length, num_trials);

trial_rel_cge = zeros(d, trial_length, num_trials);
trial_rel_be = zeros(d, trial_length, num_trials);
trial_rel_oe = zeros(d, trial_length, num_trials);

% Timing
tri_mod = 1;

%r_template = GenerateStraightRobot(world_dims);
r_template = GenerateOrbitRobot(world_dims);

N = b^(d-1);

% Visualization
truth_plotter = SequencePlotter(world_dims);
truth_plotter.z_scale = 0.1;
truth_plotter.colors = repmat([0,0,0],N,1);
truth_plotter.labels_on = false(1);

belief_plotter = HierarchyPlotter(world_dims);
belief_plotter.z_scale = 0.1;
belief_plotter.Link(truth_plotter);
belief_plotter.labels_on = false(1);
belief_plotter.colors = [0, 0, 1;
                         0, 1, 0;
                         1, 0, 0];
% 
% overlay_plotter = HierarchyPlotter(world_dims);
% overlay_plotter.colors = [0, 0, 1;
%                          0, 1, 0;
%                          1, 0, 0];
     
% Latest error plot                     
local_lines = [];                     
figure;
hold on;
local_lines(1) = plot(0, 0, 'rx-');
local_lines(2) = plot(0, 0, 'gx-');
local_lines(3) = plot(0, 0, 'bx-');
local_lines(4) = plot(0, 0, 'mx-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('k = 2', 'k = 1', 'k = 0', 'Global', 'location', 'northwest');
title('Localization Error');   

% Latest error plot                     
relative_lines = [];                     
figure;
hold on;
relative_lines(1) = plot(0, 0, 'rx-');
relative_lines(2) = plot(0, 0, 'gx-');
relative_lines(3) = plot(0, 0, 'bx-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('k = 2', 'k = 1', 'k = 0', 'Global', 'location', 'northwest');
title('Latest Relative Error');   
                     
%% Experiment loops
tic();

% Run trials
for s = 1:num_trials
    
    if mod(s, tri_mod) == 1
        now = toc();
        done = s-1;
        to_do = num_trials;
        runtime_est = now/done*to_do;
        fprintf('Trial %d/%d. ETA: %f\n', s, num_trials, runtime_est - now);
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
    %root.Initialize(sim.world.state);
    
    leafs = ChainedGraph.empty(0, numel(sim.world.robots));
    for i = 1:numel(sim.world.robots)
        leafs(i) = sim.world.robots(i).roles(end).chained_graph;
    end
    graphs = cell(d,1);
    for i = 1:numel(sim.world.robots)
        for j = 1:numel(sim.world.robots(i).roles)
            depth = sim.world.robots(i).roles(j).chained_graph.depth;
            graphs{end-depth} = [graphs{end-depth}, sim.world.robots(i).roles(j).chained_graph];
        end
    end
    
    % Run simulator and calculate errors
    r_template = sim.world.robots(1);
    r1 = sim.world.robots(2);
    r2 = sim.world.robots(3);
    r3 = sim.world.robots(4);
    
    %all_cg_errs = zeros(d, trial_length);
    
    root = sim.world.robots(1).roles(1);
    cg0_0 = sim.world.robots(1).roles(1).chained_graph;
    cg1_0 = sim.world.robots(1).roles(2).chained_graph;
    cg2_0 = sim.world.robots(1).roles(3).chained_graph;
    
    % T Scope logging
    tscope0_0 = cell(trial_length, 1);
    tscope1_0 = cell(trial_length, 1);
    tscope2_0 = cell(trial_length, 1);
    
    for i = 1:trial_length
        
        truth = sim.history(1:sim.history_ind-1);
        [cg_errs, baseline_errs, odo_errs] = CalculateLeafGraphErrors(leafs, truth);        
        trial_latest_cge(:, i, s) = mean(cg_errs, 2);
        trial_latest_be(:, i, s) = mean(baseline_errs, 2);
        trial_latest_oe(:, i, s) = mean(odo_errs, 2);
        
        eg = trial_latest_cge(4,i,s);        
        e0 = trial_latest_cge(3,i,s);
        e1 = trial_latest_cge(2,i,s);
        e2 = trial_latest_cge(1,i,s);
        
        set(local_lines(1), 'XData', 0:i-1, 'YData', trial_latest_cge(1,1:i,s));
        set(local_lines(2), 'XData', 0:i-1, 'YData', trial_latest_cge(2,1:i,s));
        set(local_lines(3), 'XData', 0:i-1, 'YData', trial_latest_cge(3,1:i,s));
        set(local_lines(4), 'XData', 0:i-1, 'YData', trial_latest_cge(4,1:i,s));
        
        fprintf('\tlocalization eg: %f e0: %f e1: %f e2: %f\n', eg, e0, e1, e2);
        
        [cg_errs, baseline_errs, odo_errs] = CalculateRelativeGraphErrors(graphs, truth);
        trial_rel_cge(:, i, s) = cg_errs;
        trial_rel_be(:, i, s) = baseline_errs;
        trial_rel_oe(:,i, s) = odo_errs;
        
        set(relative_lines(1), 'XData', 0:i-1, 'YData', trial_rel_cge(1,1:i,s));
        set(relative_lines(2), 'XData', 0:i-1, 'YData', trial_rel_cge(2,1:i,s));
        set(relative_lines(3), 'XData', 0:i-1, 'YData', trial_rel_cge(3,1:i,s));
        
        fprintf('\tlocal e0: %f e1: %f e2: %f\n', cg_errs(3), cg_errs(2), cg_errs(1));                

        truth_plotter.Clear();
        belief_plotter.Clear();
        truth_plotter.PlotSequence(sim.history(sim.history_ind-1));
        belief_plotter.PlotBeliefs(sim.world.robots(1).roles(1));
        axis(belief_plotter.axe, 'tight');
        truth_plotter.HideLines();
        truth_plotter.HideLabels();
        
%         overlay_plotter.Clear();
%         overlay_plotter.PlotTruthOverlay(root, sim.history(1:sim.history_ind-1));
%         overlay_plotter.HideLines();
%         overlay_plotter.HideLabels();
%         axis(overlay_plotter.axe, 'tight');
%         fprintf('Press any key to continue...\n');
        pause(0.1);
        
        fprintf(['Step: ', num2str(i), ' Time: ', num2str(sim.world.state.time),'\n']);
        
        sim.Step();
        n_comm = sim.comms.GetPendingSize();
        
        fprintf(['\tMessages sent: ', num2str(n_comm), '\n']);
        
        tscope0_0{i} = cg0_0.time_scope;
        tscope1_0{i} = cg1_0.time_scope;
        tscope2_0{i} = cg2_0.time_scope;
        
        n0 = numel(cg0_0.subgraph(end).measurements);
        n1 = numel(cg1_0.subgraph(end).measurements);
        n2 = numel(cg2_0.subgraph(end).measurements);
        c0 = [cg0_0.chain(1).observer_time];
        c1 = [cg1_0.chain(1).observer_time, ...
              cg1_0.chain(2).observer_time];
        c2 = [cg2_0.chain(1).observer_time, ...
              cg2_0.chain(2).observer_time, ...
              cg2_0.chain(3).observer_time];             
        fprintf(['\tCG0 T: ', num2str(cg0_0.time_scope), '\tbT: ', ...
            num2str(cg0_0.base_time), '\t|m|: ', num2str(n0), ...
            ' ct: ', num2str(c0), '\n']);
        fprintf(['\tCG1 T: ', num2str(cg1_0.time_scope), '\tbT: ', ...
            num2str(cg1_0.base_time), '\t|m|: ', num2str(n1), ...
            ' ct: ', num2str(c1), '\n']);
        fprintf(['\tCG2 T: ', num2str(cg2_0.time_scope), '\tbT: ', ...
            num2str(cg2_0.base_time), '\t|m|: ', num2str(n2), ...
            ' ct: ', num2str(c2), '\n']);
        
    end    
    
end
toc
latest_estimate_errors = mean(trial_latest_cge, 3);
latest_baseline_errors = mean(trial_latest_be, 3);
latest_odo_errors = mean(trial_latest_oe, 3);
latest_baseline_ratios = mean(trial_latest_cge./trial_latest_be, 3);
latest_odo_ratios = mean(trial_latest_cge./trial_latest_oe, 3);

relative_estimate_errors = mean(trial_rel_cge, 3);
relative_baseline_errors = mean(trial_rel_be, 3);
relative_odo_errors = mean(trial_rel_oe, 3);
relative_baseline_ratios = mean(trial_rel_cge./trial_rel_be, 3);
relative_odo_ratios = mean(trial_rel_cge./trial_rel_oe, 3);

%% Plotting
% figure;
% hold on;
% plot(0:trial_length-1, latest_baseline_ratios(1,:), 'ro-');
% plot(0:trial_length-1, latest_baseline_ratios(2,:), 'bx-');
% plot(0:trial_length-1, latest_baseline_ratios(3,:), 'g+-');
% plot(0:trial_length-1, latest_baseline_ratios(4,:), 'm^-');
% plot([0,trial_length-1], [1,1], 'k--');
% xlabel('Step number');
% ylabel('CG error/GN error');
% legend('k = 2', 'k = 1', 'k = 0', 'location', 'best');
% title('Latest Baseline Performance Ratio vs. Steps');

% figure;
% hold on;
% plot(0:trial_length-1, latest_odo_ratios(1,:), 'ro-');
% plot(0:trial_length-1, latest_odo_ratios(2,:), 'bx-');
% plot(0:trial_length-1, latest_odo_ratios(3,:), 'g+-');
% plot(0:trial_length-1, latest_odo_ratios(4,:), 'm^-');
% plot([0,trial_length-1], [1,1], 'k--');
% xlabel('Step number');
% ylabel('CG error/GN error');
% legend('k = 2', 'k = 1', 'k = 0', 'location', 'best');
% title('Latest Odometry Performance Ratio vs. Steps');

figure;
hold on;
plot(0:trial_length-1, latest_estimate_errors(1,:), 'rx-');
plot(0:trial_length-1, latest_baseline_errors(1,:), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Latest Depth 2 Estimate vs. Baseline Localized Error');

figure;
hold on;
plot(0:trial_length-1, latest_estimate_errors(2,:), 'rx-');
plot(0:trial_length-1, latest_baseline_errors(2,:), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Latest Depth 1 Estimate vs. Baseline Localized Error');

figure;
hold on;
plot(0:trial_length-1, latest_estimate_errors(3,:), 'rx-');
plot(0:trial_length-1, latest_baseline_errors(3,:), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Latest Depth 0 Estimate vs. Baseline Localized Error');

figure;
hold on;
plot(0:trial_length-1, latest_estimate_errors(4,:), 'rx-');
plot(0:trial_length-1, latest_baseline_errors(4,:), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Latest Global Frame Estimate vs. Baseline Localized Error');

figure;
hold on;
plot(0:trial_length-1, latest_estimate_errors(1,:), 'rx-');
plot(0:trial_length-1, latest_baseline_errors(1,:), 'r.-');
plot(0:trial_length-1, latest_estimate_errors(2,:), 'gx-');
plot(0:trial_length-1, latest_baseline_errors(2,:), 'g.-');
plot(0:trial_length-1, latest_estimate_errors(3,:), 'bx-');
plot(0:trial_length-1, latest_baseline_errors(3,:), 'b.-');
plot(0:trial_length-1, latest_estimate_errors(4,:), 'mx-');
plot(0:trial_length-1, latest_baseline_errors(4,:), 'm.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('k = 2 CG', 'k = 2 MLE', 'k = 1 CG', 'k = 1 MLE', ...
    'k = 0 CG', 'k = 0 MLE', 'Global CG', 'Global MLE', 'location', 'best')
title('Chained Graph vs. Baseline Localized Error');


%%

% figure;
hold on;
plot(0:trial_length-1, relative_baseline_ratios(1,:), 'ro-');
plot(0:trial_length-1, relative_baseline_ratios(2,:), 'bx-');
plot(0:trial_length-1, relative_baseline_ratios(3,:), 'g+-');
plot([0,trial_length-1], [1,1], 'k--');
xlabel('Step number');
ylabel('CG error/GN error');
legend('k = 2', 'k = 1', 'k = 0', 'location', 'best');
title('Baseline Relative Performance Ratio vs. Steps');

figure;
hold on;
plot(0:trial_length-1, relative_odo_ratios(1,:), 'ro-');
plot(0:trial_length-1, relative_odo_ratios(2,:), 'bx-');
plot(0:trial_length-1, relative_odo_ratios(3,:), 'g+-');
plot([0,trial_length-1], [1,1], 'k--');
xlabel('Step number');
ylabel('CG error/GN error');
legend('k = 2', 'k = 1', 'k = 0', 'location', 'best');
title('Odometry Relative Performance Ratio vs. Steps');

figure;
hold on;
plot(0:trial_length-1, relative_estimate_errors(2,:), 'rx-');
plot(0:trial_length-1, relative_baseline_errors(2,:), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Depth 1 Estimate and Baseline Relative Error');

figure;
hold on;
plot(0:trial_length-1, relative_estimate_errors(3,:), 'rx-');
plot(0:trial_length-1, relative_baseline_errors(3,:), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Depth 0 Estimate and Baseline Relative Error');

figure;
hold on;
plot(0:trial_length-1, relative_estimate_errors(1,:), 'rx-');
%plot(0:trial_length-1, relative_baseline_errors(1,:), 'r.-');
plot(0:trial_length-1, relative_estimate_errors(2,:), 'gx-');
%plot(0:trial_length-1, relative_baseline_errors(2,:), 'g.-');
plot(0:trial_length-1, relative_estimate_errors(3,:), 'bx-');
%plot(0:trial_length-1, relative_baseline_errors(3,:), 'b.-');
xlabel('Step number');
ylabel('Average error norm (m)');
legend('k = 2', 'k = 1', 'k = 0', 'location', 'northwest');
title('Chained Graph In-Group Average Error');

%% Plot subsampled trajectories
% 
% truth = sim.history(1:sim.history_ind-1);
% subtimes = 0:8:sim.history_ind-2;
% subtruth = truth.GetSubset([], subtimes);
% 
% subplotter = SequencePlotter(world_dims);
% subplotter.z_scale = 1;
% subplotter.SetColors(numel(sim.world.robots));
% subplotter.PlotSequence(subtruth);

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
