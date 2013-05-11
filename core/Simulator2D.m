% Simulator running and recording world
% Serves as a wrapper to the underlying World2D environment
classdef Simulator2D < handle
    
    properties
        
        world;       % Handle to sim world
        history = WorldState2D;     % Records of full state time sequence
        history_ind;
        
        vis_on;
        plotter;     % Visualization
        recorder;    % Video file ID
        recording = false;
        
        comms;  % Synchronous communication manager
        
    end
    
    methods
        
        % Create a simulation manager and simulated world.
        % Takes arguments to set world size and initialize robots
        % optionally.
        function [obj] = Simulator2D(world_size, vis_mode)
            
            if nargin == 0
                return
            end
            
            if nargin == 1
                vis_mode = true;
            end
            
            obj.world = World2D(world_size); % Initialize world
            obj.comms = SyncPostOffice(100);
            
            obj.vis_on = vis_mode;
            if vis_mode
                obj.plotter = SequencePlotter(world_size); % Initialize visualization
                obj.plotter.Label('Truth');
                obj.plotter.z_scale = 0.1;
            end
            
            obj.history(100) = WorldState2D;
            obj.history_ind = 1;
            
            if nargin < 2
                return;
            end
            
        end
        
        % Add robots to the world
        function AddRobots(obj, robots)
            
            for i = 1:numel(robots)
                robots(i).RegisterCommunications(obj.comms);
            end
            
            obj.world.AddRobots(robots);
            
            obj.history_ind = 1;
            state = obj.world.GetState();
            obj.history(obj.history_ind) = state;
            %obj.history_ind = obj.history_ind + 1;
            
            if obj.vis_on
                obj.plotter.Clear();
                obj.plotter.SetColors(obj.world.GetNumRobots());
                obj.plotter.PlotSequence(state);
            end
            
        end
        
        function [plotter] = CreatePlotter(obj)
            
            plotter = Plotter2D(1.2*obj.world.dims);
            plotter.SetColors(obj.world.GetNumRobots());
            
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
        
        % Call after adding robots and before stepping to generate first
        % state correctly
        % TODO: Fix!
        function Initialize(obj)
                        
            for i = 1:numel(obj.world.robots)
               
                obj.world.robots(i).Initialize(obj.world.state);
                
            end
            
            for i = 1:numel(obj.world.robots)
                
                obj.world.robots(i).GenerateMeasurements(obj.world.state);
                
            end
            
            obj.world.GenerateMeasurements();
            obj.history(1) = obj.world.state;
            obj.history_ind = 2;
            
        end
        
        % Proceed N time steps
        function Step(obj, N)
            
            if nargin == 1
                N = 1;
            end
            
            localHist(N) = WorldState2D;
            localInd = 1;
            
            for i = 1:N
                obj.comms.ProcessTransactions(obj.world.state.time); 
                obj.world.Step();
                                
                state = obj.world.GetState();                
                state = ChainedGraph.Compress(state);
                
                localHist(localInd) = state;
                localInd = localInd + 1;
                if obj.vis_on
                    obj.plotter.PlotSequence(state);
                end
            end                        
            
            obj.history(obj.history_ind:obj.history_ind + N - 1) = localHist;
            obj.history_ind = obj.history_ind + N;
            
        end
        
    end
    
end