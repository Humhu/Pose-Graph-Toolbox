function [r] = GenerateStraightRobot(world_dims)
    
    % Motion controller
    mc = StraightMotionController();
    mc.velocity = 1/120;
    mc.k_normal = 1;
    mc.ref = [0;0;0];
    mc.ref_direction = 0;
    
    % Motion Model
    mm = GaussianMotionModel();
    mm.mean = [0;0;0];
    % 2 cm X, 2 cm Y, 2 degree error per meter
    mm.covariance = (0.02^2)*[1,    0,  0;
        0,    1,  0;
        0,    0,  2];
    mm.input_limits = [Inf*ones(3,1), -Inf*ones(3,1)];
    mm.output_limits = [world_dims/2, -world_dims/2;
        Inf, -Inf];
    mm.output_wrapping = boolean([0,0,1]);
    
    % Sensor
    rps = RelativePoseSensor();
    rps.maxRange = 10;
    rps.mean = [0;0;0];
    % 0.5 cm X, 1 cm Y, 1 degree error per meter range
    rps.covariance = (0.005^2)*[1,   0,  0;
        0,   2,  0;
        0,   0,  4];
    
    % Robots
    r = Robot();
    r.RegisterMotionController(mc);
    r.RegisterMotionModel(mm);
    r.RegisterSensor(rps);