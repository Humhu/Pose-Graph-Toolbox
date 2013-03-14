function [cg_errs, baseline_errs, odo_errs] = ...
    CalculateLeafGraphErrors(leaf_graphs, true_graph)

N = numel(leaf_graphs);
D = leaf_graphs(1).depth;

% Initialize results
cg_errs = zeros(D + 2, N);
baseline_errs = zeros(D + 2, N);
odo_errs = zeros(D + 2, N);

% Get comparison solutions
%gn = GNSolver(1E-6, 100);
%odo = OdometrySolver(1E-6, 100);

%baseline_graph = gn.Solve(true_graph);
%odo_graph = odo.Solve(true_graph);
baseline_graph = true_graph;
odo_graph = true_graph;

for i = 1:N
   
    lg = leaf_graphs(i);
    [cg_e, b_e, o_e] = CalculateLocalGraphErrors(lg, true_graph, ...
        baseline_graph, odo_graph);    
    
    cg_errs(:, i) = cg_e;
    baseline_errs(:, i) = b_e;
    odo_errs(:, i) = o_e;
    
end

end