% Generates a world with robots and hierarchy agents
seed = 25;
world_dims = [1;1];

%% Seed random number generator
stream = RandStream('mt19937ar', 'Seed', seed);
RandStream.setGlobalStream(stream);

%% Start simulation
sim = Simulator2D(world_dims);

%% Initialize example robot

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
r0 = Robot();
r0.RegisterMotionController(mc);
r0.RegisterMotionModel(mm);
r0.RegisterSensor(rps);

%% Place robots
% Generate positions
center = zeros(3,1);
d = 3;
b = 2;
f = 0.5;
r = 0.25;
covariance = zeros(3);
seed = 0;

positions = GenerateFractal(center, b, r, f, d, covariance, seed);
positions = positions{end};
positions = [positions{:}];
positions(3,:) = wrapToPi(positions(3,:));

% Generate robots
robots = InitRobots(r0, {positions});
grouping = GenerateGrouping(d, b);
time_scales = [6;3;1];
AssignGrouping(robots, grouping, time_scales);

% Add robots to simulator
sim.AddRobots(robots);
sim.Initialize();

gn = GNSolver(1E-6, 100);
init = gn.Solve(sim.world.state);

root = sim.world.robots(1).roles(1); %hardcoded for now
root.Initialize(init); % Initializes the entire tree

%% Run simulator and calculate errors
r0 = sim.world.robots(1);
r1 = sim.world.robots(2);
r2 = sim.world.robots(3);
r3 = sim.world.robots(4);
n_steps = 20;

truth_err_avg = zeros(d, n_steps);
baseline_err_avg = zeros(d, n_steps);

% Only looking at robot 0's chained graphs for now
cg0_0 = r0.roles(1).chained_graph;
cg1_0 = r0.roles(2).chained_graph;
cg2_0 = r0.roles(3).chained_graph;
r0_cgs = [cg0_0, cg1_0, cg2_0];

for i = 1:n_steps
    
    sim.Step();
    truth = sim.history(1:sim.history_ind-1);
    [cg_errs, baseline_errs] = CalculateLocalGraphErrors(r0_cgs, truth);
    
    for j = 1:d
        
        cg_e = cg_errs{j};
        b_e = baseline_errs{j};
        truth_err_avg(j, i) = mean(sqrt(sum(cg_e.*cg_e, 1)));
        baseline_err_avg(j, i) = mean(sqrt(sum(b_e.*b_e, 1)));
    end
    
end

figure;
hold on;
plot(0:n_steps-1, truth_err_avg(1,:), 'r');
plot(0:n_steps-1, baseline_err_avg(1,:), 'b');
xlabel('Step number');
ylabel('Average error norm');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Depth 0 vs. Baseline Localized Error');

figure;
hold on;
plot(0:n_steps-1, truth_err_avg(2,:), 'r');
plot(0:n_steps-1, baseline_err_avg(2,:), 'b');
xlabel('Step number');
ylabel('Average error norm');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Depth 1 vs. Baseline Localized Error');

figure;
hold on;
plot(0:n_steps-1, truth_err_avg(2,:), 'r');
plot(0:n_steps-1, baseline_err_avg(3,:), 'b');
xlabel('Step number');
ylabel('Average error norm');
legend('Estimate', 'Global Optimum', 'location', 'best')
title('Depth 2 vs. Baseline Localized Error');

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
belief_plotter.PlotBeliefs(r0.roles(1));

truth_plotter.HideLines();
truth_plotter.HideLabels();
baseline_plotter.HideLines();
baseline_plotter.HideLabels();

