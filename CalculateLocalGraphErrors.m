function [cg_errs, baseline_errs] = CalculateLocalGraphErrors(chained_graphs, true_graph)

N = numel(chained_graphs);
cg_errs = cell(1, N);
baseline_errs = cell(1, N);

gn = GNSolver(1E-6, 100);
baseline = gn.Solve(true_graph);

for i = 1:numel(chained_graphs)
    
    cg = chained_graphs(i);
    truth = true_graph.GetSubset(cg.robot_scope, cg.time_scope);
    
    cg_truth_err = cg.CompareGraphs(truth);
    cg_baseline_err = cg.CompareGraphs(baseline);
    
    cg_errs{i} = cg_truth_err;
    baseline_errs{i} = cg_baseline_err - cg_truth_err;

end
