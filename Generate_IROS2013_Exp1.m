% Factorial testing over n, alpha with fixed random seeds

% Use same seeds for each trial per experiment
seeds = 25:25;
num_trials = numel(seeds);
trial_length = 20; % Number of steps to simulate per trial

% Test parameters
%test_time_scales = [1, 2, 3];
test_time_scales = 3;
%test_time_overlaps = [0, 1, 2];
test_time_overlaps = 1;

[time_scale_v, time_overlap_v] = ndgrid(test_time_scales, test_time_overlaps);
num_experiments = numel(time_scale_v);

estimate_errors = zeros(num_experiments, 4, trial_length);
baseline_errors = zeros(num_experiments, 4, trial_length);
error_ratios = zeros(num_experiments, 4, trial_length);

% Other fixed parameters
world_dims = [1;1];

for k = 1:num_experiments
    
    experiment_estimate_errors = zeros(num_trials, 4, trial_length);
    experiment_baseline_errors = zeros(num_trials, 4, trial_length);    
    experiment_error_ratios = zeros(num_trials, 4, trial_length);
    
    tscale = time_scale_v(k);
    toverlap = time_overlap_v(k);
    
    time_scales = [9, 3, 1];
    %time_scales = tscale.^(2:-1:0);
    time_overlaps = [toverlap*ones(1,2),0];
    
    %Initialize example robots
    % Motion controller
    mc = OrbitMotionController();
    mc.ref = 0; %TODO: unused
    mc.motionGain = 0.1;
    
    % Motion Model
    mm = GaussianMotionModel();
    mm.mean = [0;0;0];
    %mm.covariance = (0.02)^2*eye(3);
    mm.covariance = (0.01^2)*[1,    0,  0;
        0,    1,  0;
        0,    0,  2];
    mm.input_limits = [Inf*ones(3,1), -Inf*ones(3,1)];
    mm.output_limits = [world_dims/2, -world_dims/2;
        Inf, -Inf];
    mm.output_wrapping = boolean([0,0,1]);
    
    % Sensor
    rps = RelativePoseSensor();
    rps.maxRange = 0.6;
    rps.mean = [0;0;0];
    rps.covariance = (0.005^2)*[1,   0,  0;
        0,   1,  0;
        0,   0,  2];
    
    % Robots
    r_template = Robot();
    r_template.RegisterMotionController(mc);
    r_template.RegisterMotionModel(mm);
    r_template.RegisterSensor(rps);
    
    % Place robots
    % Generate positions
    center = zeros(3,1);
    d = 3;
    b = 2;
    f = 0.5;
    r = 0.25;
    covariance = zeros(3);       
    
    for s = 1:num_trials

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
        
        AssignGrouping(robots, grouping, time_scales, time_overlaps); % Set up hierarchy and assign roles
        
        % Add robots to simulator
        sim.AddRobots(robots);
        sim.Initialize();
        
        gn = GNSolver(1E-6, 100);
        init = gn.Solve(sim.world.state);
        
        root = sim.world.robots(1).roles(1); %hardcoded for now
        root.Initialize(init); % Initializes the entire tree
        
        % Run simulator and calculate errors
        r_template = sim.world.robots(1);
        r1 = sim.world.robots(2);
        r2 = sim.world.robots(3);
        r3 = sim.world.robots(4);        

        trial_estimate_errors = zeros(d+1, trial_length);
        trial_baseline_errors = zeros(d+1, trial_length);        
        
        % Only looking at robot 0's chained graphs for now
        cg0_0 = r_template.roles(1).chained_graph;
        cg1_0 = r_template.roles(2).chained_graph;
        cg2_0 = r_template.roles(3).chained_graph;
        r0_cgs = [cg0_0, cg1_0, cg2_0];

        for i = 1:trial_length

            sim.Step();
            truth = sim.history(1:sim.history_ind-1);
            [cg_errs, baseline_errs] = CalculateLocalGraphErrors(cg2_0, truth);

            for j = 1:d+1

                cg_e = cg_errs{j};
                cg_e(3,:) = cg_e(3,:)/(2*pi);
                b_e = baseline_errs{j};
                b_e(3,:) = b_e(3,:)/(2*pi);
                trial_estimate_errors(j, i) = mean(sqrt(sum(cg_e.*cg_e, 1)));
                trial_baseline_errors(j, i) = mean(sqrt(sum(b_e.*b_e, 1)));
            end

        end
        trial_error_ratios = trial_estimate_errors./trial_baseline_errors;
        
        experiment_estimate_errors(s,:,:) = trial_estimate_errors;
        experiment_baseline_errors(s,:,:) = trial_baseline_errors;
        experiment_error_ratios(s,:,:) = trial_error_ratios;
    
    end
        
    estimate_errors(k,:,:) = mean(experiment_estimate_errors, 1);
    baseline_errors(k,:,:) = mean(experiment_baseline_errors, 1);
    error_ratios(k,:,:) = mean(experiment_error_ratios, 1);
    
end

%% Plotting    
figure;
hold on;
plot(0:trial_length-1, squeeze(error_ratios(1,1,:)), 'ro-');
plot(0:trial_length-1, squeeze(error_ratios(1,2,:)), 'bx-');
plot(0:trial_length-1, squeeze(error_ratios(1,3,:)), 'g+-');
plot([0,trial_length-1], [1,1], 'k--');
%plot(0:n_step-1, est_baseline_ratio(1,:), 'r');
xlabel('Step number');
ylabel('Estimate error/Baseline error');
legend('Depth 2', 'Depth 1', 'Depth 0', 'location', 'best');
title('Baseline Performance Ratio vs. Steps');

% figure;
% hold on;
% plot(0:num_trials-1, truth_err_avg(1,:), 'r');
% plot(0:num_trials-1, baseline_err_avg(1,:), 'b');
% xlabel('Step number');
% ylabel('Average error norm');
% legend('Estimate', 'Global Optimum', 'location', 'best')
% title('Depth 2 Estimate vs. Baseline Localized Error');
% 
% figure;
% hold on;
% plot(0:num_trials-1, truth_err_avg(2,:), 'r');
% plot(0:num_trials-1, baseline_err_avg(2,:), 'b');
% xlabel('Step number');
% ylabel('Average error norm');
% legend('Estimate', 'Global Optimum', 'location', 'best')
% title('Depth 1 Estimate vs. Baseline Localized Error');
% 
% figure;
% hold on;
% plot(0:num_trials-1, truth_err_avg(3,:), 'r');
% plot(0:num_trials-1, baseline_err_avg(3,:), 'b');
% xlabel('Step number');
% ylabel('Average error norm');
% legend('Estimate', 'Global Optimum', 'location', 'best')
% title('Depth 0 Estimate vs. Baseline Localized Error');
% 
% figure;
% hold on;
% plot(0:num_trials-1, truth_err_avg(4,:), 'r');
% plot(0:num_trials-1, baseline_err_avg(4,:), 'b');
% xlabel('Step number');
% ylabel('Average error norm');
% legend('Estimate', 'Global Optimum', 'location', 'best')
% title('Global Frame Estimate vs. Baseline Localized Error');

truth = sim.history(1:sim.history_ind-1);

% Get comparison estimate
gn = GNSolver(1E-6, 100);
baseline_est = gn.Solve(truth);

% Plot results
truth_plotter = SequencePlotter(world_dims);
truth_plotter.z_scale = 0.1;
truth_plotter.colors = repmat([0,1,0],4,1);
truth_plotter.PlotSequence(truth);

baseline_plotter = SequencePlotter(world_dims);
baseline_plotter.z_scale = 0.1;
baseline_plotter.Link(truth_plotter);
baseline_plotter.colors = repmat([0,0,1],4,1);
baseline_plotter.PlotSequence(baseline_est);

belief_plotter = HierarchyPlotter(world_dims);
belief_plotter.z_scale = 0.1;
belief_plotter.Link(truth_plotter);
belief_plotter.colors = repmat([1,0,0],4,1);
belief_plotter.PlotBeliefs(sim.world.robots(1).roles(1));

truth_plotter.HideLines();
truth_plotter.HideLabels();
baseline_plotter.HideLines();
baseline_plotter.HideLabels();

