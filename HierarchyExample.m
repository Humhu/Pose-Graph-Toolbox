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
mm.covariance = [0.02.^2,    0,      0;
                0,         0.02^2, 0;
                0,         0,      0.04^2];
mm.input_limits = [Inf*ones(3,1), -Inf*ones(3,1)];
mm.output_limits = [world_dims/2, -world_dims/2;
    Inf, -Inf];
mm.output_wrapping = boolean([0,0,1]);

% Sensor
rps = RelativePoseSensor();
rps.maxRange = Inf;
rps.mean = [0;0;0];
rps.covariance = [0.01^2,   0,      0;
                0,        0.01^2, 0;
                0,        0,      0.04^2];

% Robots
r0 = Robot();
r0.RegisterMotionController(mc);
r0.RegisterMotionModel(mm);
r0.RegisterSensor(rps);

%% Place robots
% Generate positions
center = zeros(3,1);
d = 3;
b = 3;
f = 0.4;
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
time_scales = [9;3;1];
AssignGrouping(robots, grouping, time_scales);

% Add robots to simulator
sim.AddRobots(robots);

root = sim.world.robots(1).roles(1); %hardcoded for now
root.Initialize(sim.world.state); % Initializes the entire tree

%% Set up hierarchy
% TODO: Programmatic way to initialize hierarchy roles...
% Roles (3 robots for now = 4 roles)
% root = HierarchyRole(0);
% leaf0 = HierarchyRole(1);
% leaf1 = HierarchyRole(1);
% leaf2 = HierarchyRole(1);
% root.AssignFollowers([leaf0, leaf1, leaf2]);
% 
% leaf0.time_scale = 1;
% leaf1.time_scale = 1;
% leaf2.time_scale = 1;
% root.time_scale = 3;
% 
% Register roles
% robots(1).RegisterRole(root);
% robots(1).RegisterRole(leaf0);
% robots(2).RegisterRole(leaf1);
% robots(3).RegisterRole(leaf2);
% 
% Initialize roles
% s0r = sim.world.state;
% s0r.poses = bsxfun(@minus, s0r.poses, s0r.poses(:,1));
% s0r.poses(3,:) = wrapToPi(s0r.poses(3,:));
% 
% s0 = WorldState2D;
% s0.ids = robots(1).ID;
% s0.poses = zeros(3,1);
% s0.time = 0;
% 
% s1 = WorldState2D;
% s1.ids = robots(2).ID;
% s1.poses = zeros(3,1);
% s1.time = 0;
% 
% s2 = WorldState2D;
% s2.ids = robots(3).ID;
% s2.poses = zeros(3,1);
% s2.time = 0;
% 
% root.Initialize(s0r);
% leaf0.Initialize(s0);
% leaf1.Initialize(s1);
% leaf2.Initialize(s2);
