function [cg_errs, baseline_errs, odo_errs] = ...
    CalculateAllGraphErrors(all_graphs, true_graph)

D = numel(all_graphs);

% Initialize results
cg_errs = zeros(D, 1);
baseline_errs = zeros(D, 1);
odo_errs = zeros(D, 1);

% Get comparison solutions
gn = GNSolver(1E-6, 100);
odo = OdometrySolver(1E-6, 100);

baseline_graph = gn.Solve(true_graph);
odo_graph = odo.Solve(true_graph);

for i = 1:D
   
    graphs = all_graphs{i};
    cg_err = zeros(1, numel(graphs));
    b_err = zeros(1, numel(graphs));
    o_err = zeros(1, numel(graphs));
    
    for j = 1:numel(graphs)
        
        cg = graphs(j);
        
        N = numel(cg.robot_scope);
        %[idMap, tMap] = cg.subgraph.BuildMaps();                                
        
        cg_truth_err = cg.CompareGraphs(true_graph);
        cg_baseline_err = cg.CompareGraphs(baseline_graph);
        cg_odo_err = cg.CompareGraphs(odo_graph);
        
        cg_e = cg_truth_err;
        b_e = cg_truth_err - cg_baseline_err;
        o_e = cg_truth_err - cg_odo_err;
        
        cg_e = cg_e(:, end-N+1:end);
        b_e = b_e(:,end-N+1:end);
        o_e = o_e(:,end-N+1:end);
        
        cg_err(i) = mean(sqrt(sum(cg_e.*cg_e, 1)),2);
        b_err(i) = mean(sqrt(sum(b_e.*b_e, 1)),2);
        o_err(i) = mean(sqrt(sum(o_e.*o_e, 1)),2);
        
    end
    cg_errs(i) = mean(cg_err, 2);
    baseline_errs(i) = mean(b_err, 2);
    odo_errs(i) = mean(o_err, 2);
    
end

end