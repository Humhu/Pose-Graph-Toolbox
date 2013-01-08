% Generates a world with stationary landmarks and a single robot
seed = 25;

% Initialize example landmark
l = Robot();

lmc = StationaryMotionController();
l.RegisterMotionController(lmc);

lmm = GaussianMotionModel();
lmm.mean = [0;0;0];
lmm.covariance = 1E-16*eye(3); % Can't use 0 covariance because it blows up
l.RegisterMotionModel(lmm);

% Initialize example robot
r = Robot();

mc = OrbitMotionController();
mc.ref = 0; %TODO: unused
mc.motionGain = 0.1;
r.RegisterMotionController(mc);

mm = GaussianMotionModel();
mm.mean = [0;0;0];
%mm.covariance = (0.02)^2*eye(3);
mm.covariance = [0.02.^2,    0,      0;
                 0,         0.02^2, 0;
                 0,         0,      0.04^2];
r.RegisterMotionModel(mm);

rps = RelativePoseSensor();
rps.maxRange = 0.5;
rps.mean = [0;0;0];
rps.covariance = [0.01^2,   0,      0;
                  0,        0.01^2, 0;
                  0,        0,      0.04^2];
r.RegisterSensor(rps);

% Seed random number generator
stream = RandStream('mt19937ar', 'Seed', seed);
RandStream.setGlobalStream(stream);

% Start simulation
sim = Simulator2D([1,1]);
sim.InitRobots(1, l);
sim.InitRobots(3, r);