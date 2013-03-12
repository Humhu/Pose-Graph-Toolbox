function [cg_errs, baseline_errs, odo_errs] = ...
    CalculateLocalGraphErrors(chained_graph, true_graph, baseline_graph, odo_graph)

vis_on = false;
    
D = chained_graph.depth;
cg_errs = zeros(D + 2, 1);
baseline_errs = zeros(D + 2, 1);
odo_errs = zeros(D + 2, 1);

gn = GNSolver(1E-6, 100);
odo = OdometrySolver(1E-6, 100);

if nargin == 2

    baseline_graph = gn.Solve(true_graph);
    odo_graph = odo.Solve(true_graph);
    
end

if vis_on
    % %Visualize shifted graphs
    world_dims = [30,20];
    truth_plotter = SequencePlotter(world_dims);
    truth_plotter.z_scale = 0.1;
    truth_plotter.colors = repmat([0,1,0],4,1);
    
    baseline_plotter = SequencePlotter(world_dims);
    baseline_plotter.z_scale = 0.1;
    baseline_plotter.Link(truth_plotter);
    baseline_plotter.colors = repmat([0,0,1],4,1);
    
    odo_plotter = SequencePlotter(world_dims);
    odo_plotter.z_scale = 0.1;
    odo_plotter.Link(truth_plotter);
    odo_plotter.colors = repmat([1,0,1],4,1);
    
    belief_plotter = SequencePlotter(world_dims);
    belief_plotter.z_scale = 0.1;
    belief_plotter.Link(truth_plotter);
    belief_plotter.colors = repmat([1,0,0],4,1);
end

for i = 1:D + 2
    
    translated_graph = chained_graph.ApplyLinks(i - 1);
    
    cg_truth_err = translated_graph.CompareGraphs(true_graph);
    cg_baseline_err = translated_graph.CompareGraphs(baseline_graph);
    cg_odo_err = translated_graph.CompareGraphs(odo_graph);
    
    cg_truth_err = cg_truth_err(:, end);
    cg_baseline_err = cg_baseline_err(:, end);
    cg_odo_err = cg_odo_err(:, end);
    
    if vis_on
        truth_plotter.Clear();
        baseline_plotter.Clear();
        odo_plotter.Clear();
        belief_plotter.Clear();
        shifted_truth = true_graph.Zero(translated_graph.base_id, translated_graph.base_time);
        shifted_truth = shifted_truth.GetSubset(translated_graph.robot_scope, translated_graph.time_scope);
        shifted_baseline = baseline_graph.Zero(translated_graph.base_id, translated_graph.base_time);
        shifted_baseline = shifted_baseline.GetSubset(translated_graph.robot_scope, translated_graph.time_scope);
        shifted_odo = odo_graph.Zero(translated_graph.base_id, translated_graph.base_time);
        shifted_odo = shifted_odo.GetSubset(translated_graph.robot_scope, translated_graph.time_scope);
        
        truth_plotter.PlotSequence(shifted_truth);
        baseline_plotter.PlotSequence(shifted_baseline);
        odo_plotter.PlotSequence(shifted_odo);
        belief_plotter.PlotSequence(translated_graph.subgraph);
        
        truth_plotter.HideLines();
        truth_plotter.HideLabels();
        baseline_plotter.HideLines();
        baseline_plotter.HideLabels();
        odo_plotter.HideLines();
        odo_plotter.HideLabels();
    end

    cg_e = cg_truth_err;
    b_e = cg_truth_err - cg_baseline_err;
    o_e = cg_truth_err - cg_odo_err;
    
    cg_errs(i,:) = mean(sqrt(sum(cg_e.*cg_e, 1)),2);
    baseline_errs(i,:) = mean(sqrt(sum(b_e.*b_e, 1)),2);
    odo_errs(i,:) = mean(sqrt(sum(o_e.*o_e, 1)),2);
    
end
