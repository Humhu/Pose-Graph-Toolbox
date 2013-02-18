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
mc = StationaryMotionController();
mc.ref = [-0.1; 0; 0];

% Motion Model
mm = GaussianMotionModel();
mm.mean = [0;0;0];
%mm.covariance = (0.02)^2*eye(3);
mm.covariance = 3*[0.02.^2,    0,      0;
                0,         0.02^2, 0;
                0,         0,      0.04^2];
mm.input_limits = [Inf*ones(3,1), -Inf*ones(3,1)];
mm.output_limits = [world_dims/2, -world_dims/2;
    Inf, -Inf];
mm.output_wrapping = boolean([0,0,1]);

% Sensor
rps = RelativePoseSensor();
rps.maxRange = 0.4;
rps.mean = [0;0;0];
rps.covariance = [0.01^2,   0,      0;
                0,        0.01^2, 0;
                0,        0,      0.04^2];

% Robots
r0 = Robot();
r0.ID = 0;
r0.RegisterMotionController(mc);
r0.RegisterMotionModel(mm);
r0.RegisterSensor(rps);

%% Landmarks
mc = StationaryMotionController();
mc.ref = zeros(3,1);

mm = GaussianMotionModel(mm);
mm.mean = zeros(3,1);
mm.covariance = 1E-12*eye(3);

l = Robot();
l.ID = 1;
l.RegisterMotionController(mc);
l.RegisterMotionModel(mm);
l.RegisterSensor(rps);

%% Place robots
% Generate positions
center = zeros(3,1);
d = 2;
b = 4;
f = 0.5;
r = 0.25;
covariance = zeros(3);
seed = 0;

positions = GenerateFractal(center, b, r, f, d, covariance, seed);
positions = positions{end};
positions = [positions{:}];
positions(3,:) = wrapToPi(positions(3,:));

robot_start = positions(:,1);
landmark_start = positions(:,2:end);

% Generate robots
landmarks = InitRobots(l, {landmark_start});
r0.pose = robot_start;

% Add robots to simulator
sim.AddRobots([landmarks, r0]);
