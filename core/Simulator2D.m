% Simulator running and recording world
classdef Simulator2D < handle
    
    properties
        
        world;       % Handle to sim world
        history;     % Records of full state time sequence
        plotter;     % Visualization
        recorder;    % Video file ID
        recording = false;
        em;
        em_plotter;        
        
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
            
            %obj.em = EMIterate(100); % TODO: Un-hardcode!
            obj.em = GNIterate(100);
            obj.em_plotter = Plotter2D(world_size);
            obj.em_plotter.Label('Estimate');
            
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
                %r.pose = Pose2D(positions(i,:,:),orientations(i));
                r.pose = poses(:,i);
                r.SetID(id);
                id = id + 1;
                
            end
            obj.world.AddRobots(robs);
            state = obj.world.GetState();
            
            obj.history = Sequence2D(1);
            obj.history.Write(state);
            
            obj.em.Initialize(state);
            obj.em_plotter.SetColors(obj.world.GetNumRobots());
            obj.EMVisualize();
            
            obj.plotter.Clear();
            obj.plotter.SetColors(obj.world.GetNumRobots());
            obj.plotter.PlotState(state);
            
        end
        
        function [] = BeginRecording(obj, fname)
            if obj.recording
                obj.recorder.close()
            end
            obj.recorder = VideoWriter([fname, '.avi'], 'Motion JPEG AVI');
            obj.recorder.FrameRate = 24;
            obj.recorder.open();
            obj.recording = true;
            axis(obj.em_plotter.axe, [-0.5, 0.5, -0.5, 0.5, 0, 1]);
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
            
            localHist = Sequence2D(N);
            
            for i = 1:N
                obj.world.Step();
                state = obj.world.GetState();
                localHist.Write(state);
                obj.plotter.PlotState(state);
                obj.em.Update(state);
            end
            
            obj.history = obj.history.Append(localHist);
            
        end
        
        function [errs] = StepSolve(obj,N)
            
            if nargin == 1
                N = 1;
            end                        
            
            errs = cell(1,N);
            
            for i = 1:N
                obj.Step();
                errs{i} = obj.RunEM(true);
            end
            
        end
        
        function [errs] = RunEM(obj, vis)
            
            max_iters = 100;
            tol = 1E-3;
            
            for i = 1:max_iters
                [dMax, dNorm] = obj.em.Iterate();
                fprintf(['Iteration: ', num2str(i), '\tDelta max: ', num2str(dMax), '\tDelta norm: ', num2str(dNorm), '\n']);
                if vis
                    obj.EMVisualize();
                end
                if obj.recording
                    t = obj.world.state.time;
                    if t > 8
                       axis([-0.5, 0.5, -0.5, 0.5, t*0.1 - 0.8, t*0.1 + 0.2]); 
                    end
                    f = getframe(obj.em_plotter.fig);
                    obj.recorder.writeVideo(f);
                end
                if dNorm < tol
                    break
                end
                
            end
            
            errs = obj.em.truth.Difference(obj.em.beliefs);
            fprintf('Errors (n, t):\n');
            disp(errs);
            
            obj.EMVisualize();
            
        end
        
        function [] = EMVisualize(obj)
            
            belief = obj.em.beliefs;
            history = obj.history;
            
            obj.em_plotter.Clear();
            obj.em_plotter.PlotSequence(belief);
            t = belief.GetLength();
            if ~isempty(obj.em.covariance)
                if t > 10
                   belief = belief.Subset(t-10:t);
                   history = history.Subset(t-10:t);
                end                
                obj.em_plotter.PlotSequenceCovariances(history, belief, obj.em.covariance);
            end
            
        end
        
    end
    
    
    
    
    
    
    
    
    
end