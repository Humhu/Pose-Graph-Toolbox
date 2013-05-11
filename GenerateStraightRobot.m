function [r] = GenerateStraightRobot(world_dims)
    
    update_rate = 40; % Hertz

    % Motion controller
    mc = StraightMotionController();
    mc.velocity = 1/update_rate;
    mc.k_normal = 1;
    mc.ref = [0;0;0];
    mc.ref_direction = 0;
    
    % Motion Model
    mm = GaussianMotionModel();
    mm.mean = [0;0;0];
    % 5 cm X, 5 cm Y, 5 degree error per meter (5%)
    mm.covariance = (0.05^2)*[1,    0,  0;
                                0,    1,  0;
                                0,    0,  2];
    mm.input_limits = [Inf*ones(3,1), -Inf*ones(3,1)];
    mm.output_limits = [world_dims/2, -world_dims/2;
        Inf, -Inf];
    mm.output_wrapping = boolean([0,0,1]);
    
    % Sensor
    rps = RelativePoseSensor();
    rps.maxRange = 5;
    rps.mean = [0;0;0];
    % 1 cm X, 3 cm Y, 2 degree error per meter range (1%)
    rps.covariance = (0.01^2)*[1,   0,  0;
                                0,   3,  0;
                                0,   0,  4];
    
    % Robots
    r = Robot();
    r.RegisterMotionController(mc);
    r.RegisterMotionModel(mm);
    r.RegisterSensor(rps);