% Simulator running and recording world
classdef Simulator2D < handle
    
    properties
        
        world;       % Handle to sim world
        history;     % Records of full state time sequence
        plotter;     % Visualization
        
    end
    
    methods
        
        % Create a simulation manager and simulated world.
        % Takes arguments to set world size and initialize robots
        % optionally.
        function [obj] = Simulator2D(world_size, num_robots)
            
            if nargin == 0
                return
            end
            
            obj.world = World2D(world_size); % Initialize world
            obj.history = Sequence2D();      % Initialize recorder
            obj.plotter = Plotter2D(world_size); % Initialize visualization            
            
            if nargin < 2
                return
            end
            
            obj.InitRobots(num_robots);
            
            
        end
        
        % Given a number, initialize default robots and adds them to the world
        % Alternatively, copies a given robot and adds N of them to the
        % world
        % Returns handles of initialized robots
        function [robs] = InitRobots(obj, N, r)
            
            robs = Robot;
            
            if nargin == 2
                
                r = Robot();
                
                mc = OrbitController();
                mc.ref = 0; %TODO: unused
                mc.motionGain = 0.1;
                mm = GaussianMotionModel();
                mm.mean = zeros(3,1);
                mm.covariance = zeros(3);
                r.motionController = mc;
                r.motionModel = mm;
                
                r.sensor_range = Inf;
                r.sensor_mean = [0;0;0];
                r.sensor_covariance = 0.001*eye(3);
                r.sensor_type = 'RelativePose';     
            
            end
            
            for i = 1:N
                robs(i,1) = Robot(r);
            end

            
            dim_scale = reshape(obj.world.dims/2, 1, 1, 2);
            positions = bsxfun(@times, dim_scale, 2*rand(N,1,2) - 1);
            orientations = 2*pi*rand(N,1);                        
            
            for i = 1:N
                r = robs(i);
                r.pose = Pose2D(positions(i,:,:),orientations(i));
                r.id = i;
                           
            end                        
            obj.world.AddRobots(robs);
            state = obj.world.GetState();            
            
            obj.plotter.SetColors(N);
            obj.plotter.PlotState(state);
            
        end
        
        
        
        function [] = Step(obj)
            
            obj.world.Step();
            state = obj.world.GetState();
            obj.plotter.PlotState(state);
            
        end                
        
    end
    
    
    
    
    
    
    
    
    
end