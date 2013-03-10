function [cg_errs, baseline_errs] = CalculateLocalGraphErrors(chained_graph, true_graph)

%N = numel(chained_graph);
D = chained_graph.depth;
cg_errs = cell(1, D + 2);
baseline_errs = cell(1, D + 2);

gn = GNSolver(1E-6, 100);
baseline_graph = gn.Solve(true_graph);

%Visualize shifted graphs
% world_dims = [2,2];
% truth_plotter = SequencePlotter(world_dims);
% truth_plotter.z_scale = 0.1;
% truth_plotter.colors = repmat([0,1,0],4,1);
% 
% baseline_plotter = SequencePlotter(world_dims);
% baseline_plotter.z_scale = 0.1;
% baseline_plotter.Link(truth_plotter);
% baseline_plotter.colors = repmat([0,0,1],4,1);
% 
% belief_plotter = SequencePlotter(world_dims);
% belief_plotter.z_scale = 0.1;
% belief_plotter.Link(truth_plotter);
% belief_plotter.colors = repmat([1,0,0],4,1);

for i = 1:D + 2
    
    translated_graph = chained_graph.ApplyLinks(i - 1);
    
    cg_truth_err = translated_graph.CompareGraphs(true_graph);
    cg_baseline_err = translated_graph.CompareGraphs(baseline_graph);
    
%     truth_plotter.Clear();
%     baseline_plotter.Clear();
%     belief_plotter.Clear();
%     shifted_truth = true_graph.Zero(translated_graph.base_id, translated_graph.base_time);
%     shifted_truth = shifted_truth.GetSubset(translated_graph.robot_scope, translated_graph.time_scope);
%     shifted_baseline = baseline_graph.Zero(translated_graph.base_id, translated_graph.base_time);
%     shifted_baseline = shifted_baseline.GetSubset(translated_graph.robot_scope, translated_graph.time_scope);
%     truth_plotter.PlotSequence(shifted_truth);
%     baseline_plotter.PlotSequence(shifted_baseline);
%     belief_plotter.PlotSequence(translated_graph.subgraph);    
%     
%     truth_plotter.HideLines();
%     truth_plotter.HideLabels();
%     baseline_plotter.HideLines();
%     truth_plotter.HideLines();
    
    cg_errs{i} = cg_truth_err;
    baseline_errs{i} = cg_truth_err - cg_baseline_err;

end
