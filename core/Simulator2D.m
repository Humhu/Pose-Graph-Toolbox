% Simulator running and recording world
classdef Simulator2D < handle
    
    properties
        
        world;       % Handle to sim world
        history = WorldState2D;     % Records of full state time sequence
        history_ind;
        
        plotter;     % Visualization
        recorder;    % Video file ID
        recording = false;        
        
    end
    
    methods
        
        % Create a simulation manager and simulated world.
        % Takes arguments to set world size and initialize robots
        % optionally.
        function [obj] = Simulator2D(world_size, num_robots, seed)
            
            if nargin == 0
                return
            end
            
            if nargin == 3
                stream = RandStream('mt19937ar', 'Seed', seed);
                RandStream.setGlobalStream(stream);
            end
            
            obj.world = World2D(world_size); % Initialize world
            obj.plotter = Plotter2D(world_size); % Initialize visualization
            obj.plotter.Label('Truth');
            
            obj.history(100) = WorldState2D;
            obj.history_ind = 1;
            
            if nargin < 2
                return;
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
                
                mc = OrbitMotionController();
                mc.ref = 0; %TODO: unused
                mc.motionGain = 0.1;
                r.RegisterMotionController(mc);
                
                mm = GaussianMotionModel();
                mm.mean = [0;0;0];
                mm.covariance = (0.01)^2*eye(3);
                mm.input_limits = [Inf*ones(3,1), -Inf*ones(3,1)];
                mm.output_limits = [obj.world.dims/2, -obj.world.dims/2;
                                    Inf, -Inf];
                mm.output_wrapping = boolean([0,0,1]);
                r.RegisterMotionModel(mm);
                
                rps = RelativePoseSensor();
                rps.maxRange = 0.5;
                rps.mean = [0;0;0];
                rps.covariance = (0.01)^2*eye(3);
                r.RegisterSensor(rps);
                
            end            
            
            for i = 1:N
                robs(i,1) = Robot(r);
            end
            
            
            dim_scale = obj.world.dims/2;
            positions = bsxfun(@times, dim_scale, 2*rand(2,N) - 1);
            orientations = wrapToPi(2*pi*rand(1,N));
            poses = [positions;
                orientations];
            id = obj.world.GetNumRobots() + 1;
            for i = 1:N
                
                r = robs(i);               
                r.pose = poses(:,i);
                r.SetID(id);
                id = id + 1;
                
            end
            obj.world.AddRobots(robs);
            state = obj.world.GetState();

            obj.history(obj.history_ind) = state;
            obj.history_ind = obj.history_ind + 1;
            
            obj.plotter.Clear();
            obj.plotter.SetColors(obj.world.GetNumRobots());
            obj.plotter.PlotState(state);
            
        end
        
        % TODO: Move recording methods to Plotter2D
        function [] = BeginRecording(obj, fname)
            if obj.recording
                obj.recorder.close()
            end
            obj.recorder = VideoWriter([fname, '.avi'], 'Motion JPEG AVI');
            obj.recorder.FrameRate = 24;
            obj.recorder.open();
            obj.recording = true;
            axis(obj.plotter.axe, [-0.5, 0.5, -0.5, 0.5, 0, 1]);
        end
        
        function [] = StopRecording(obj)
            if obj.recording
                obj.recorder.close()
            end
            obj.recording = false;
        end
        
        % Proceed N time steps
        function [] = Step(obj, N)
            
            if nargin == 1
                N = 1;
            end
            
            localHist(N) = WorldState2D;
            localInd = 1;
            
            for i = 1:N
                obj.world.Step();
                state = obj.world.GetState();
                localHist(localInd) = state;
                localInd = localInd + 1;
                obj.plotter.PlotState(state);
            end
            
            obj.history(obj.history_ind:obj.history_ind + N - 1) = localHist;
            obj.history_ind = obj.history_ind + N;
            
        end       
                
    end    
    
end