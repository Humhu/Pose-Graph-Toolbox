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
d = 2;
b = 3;
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
time_scales = [3;1];
AssignGrouping(robots, grouping, time_scales);

% Add robots to simulator
sim.AddRobots(robots);

root = sim.world.robots(1).roles(1); %hardcoded for now
root.Initialize(sim.world.state); % Initializes the entire tree

%% Run simulator and calculate errors
sim.Step(10);

r0 = sim.world.robots(1);
r1 = sim.world.robots(2);
r2 = sim.world.robots(3);

truth = sim.history(1:sim.history_ind-1);

% Get comparison estimate
gn = GNSolver(1E-6, 100);
baseline_est = gn.Solve(truth);

% Plot results
truth_plotter = SequencePlotter(world_dims);
truth_plotter.z_scale = 0.1;
truth_plotter.colors = repmat([0,1,0],3,1);
truth_plotter.PlotSequence(truth);

baseline_plotter = SequencePlotter(world_dims);
baseline_plotter.z_scale = 0.1;
baseline_plotter.Link(truth_plotter);
baseline_plotter.colors = repmat([0,0,1],3,1);
baseline_plotter.PlotSequence(baseline_est);

belief_plotter = HierarchyPlotter(world_dims);
belief_plotter.z_scale = 0.1;
belief_plotter.Link(truth_plotter);
belief_plotter.colors = repmat([1,0,0],3,1);
belief_plotter.PlotBeliefs(r0.roles(1));

% k = 1 errors


