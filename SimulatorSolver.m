% Pulls measurements out of a simulator and solves for state
% Represents a centralized solving approach - useful for comparison
classdef SimulatorSolver < handle
    
    properties
        
        simulator;      % Respective simulator
        
        belief;         % Latest beliefs
        belief_ind;
        belief_cov;     % Belief covariances
        
        solver;         % GN batch solver
        
    end
    
    methods
        
        function [obj] = SimulatorSolver(sim)
            
            if nargin == 0
                return
            end
            
            obj.simulator = sim;
            obj.belief(100) = WorldState2D;
            obj.belief_ind = 1;
            
            obj.solver = GNSolver(1E-3, 100); %tolerance, max iterations
            
        end
        
        function Initialize(obj, initial)
           
            obj.belief(1) = initial;
            obj.belief_ind = 2;
            
        end
        
        function [] = Update(obj, state)
            
            % Initialize guess for new poses as previous poses
            state.poses = obj.belief(obj.belief_ind - 1).poses;
            obj.belief(obj.belief_ind) = state;                        
            obj.belief_ind = obj.belief_ind + 1;
            
            [obj.belief, obj.belief_cov] = obj.solver.Solve(obj.belief);
            
        end
        
%         % TODO: Break up and update
%         function [errs] = RunEM(obj, vis)
%             
%             % Out of place
%             %obj.em_plotter.SetColors(obj.world.GetNumRobots());
%             %obj.EMVisualize();
%             
%             max_iters = 100;
%             tol = 1E-3;
%             
%             for i = 1:max_iters
%                 [dMax, dNorm] = obj.em.Iterate();
%                 fprintf(['Iteration: ', num2str(i), '\tDelta max: ', num2str(dMax), '\tDelta norm: ', num2str(dNorm), '\n']);
%                 if vis
%                     obj.EMVisualize();
%                 end
%                 if obj.recording
%                     t = obj.world.state.time;
%                     if t > 8
%                         axis([-0.5, 0.5, -0.5, 0.5, t*0.1 - 0.8, t*0.1 + 0.2]);
%                     end
%                     f = getframe(obj.em_plotter.fig);
%                     obj.recorder.writeVideo(f);
%                 end
%                 if dNorm < tol
%                     break
%                 end
%                 
%             end
%             
%             errs = obj.em.truth.Difference(obj.em.beliefs);
%             fprintf('Errors (n, t):\n');
%             disp(errs);
%             
%             obj.EMVisualize();
%             
%         end
%         
%     end
%     
%     function [] = EMVisualize(obj)
%     
%     belief = obj.em.beliefs;
%     history = obj.history;
%     
%     obj.em_plotter.Clear();
%     obj.em_plotter.PlotSequence(belief);
%     t = belief.GetLength();
%     if ~isempty(obj.em.covariance)
%         if t > 10
%             belief = belief.Subset(t-10:t);
%             history = history.Subset(t-10:t);
%         end
%         %obj.em_plotter.PlotSequenceCovariances(history, belief,
%         %obj.em.covariance);
%     end
    
    end
    
end





