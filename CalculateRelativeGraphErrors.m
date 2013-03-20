function [cg_errs, baseline_errs, odo_errs] = ...
    CalculateRelativeGraphErrors(all_graphs, true_graph)

D = numel(all_graphs);

% Initialize results
cg_errs = zeros(D, 1);
baseline_errs = zeros(D, 1);
odo_errs = zeros(D, 1);

% Get comparison solutions
gn = GNSolver(1E-6, 100);
odo = OdometrySolver(1E-6, 100);

%baseline_graph = gn.Solve(true_graph);
%odo_graph = odo.Solve(true_graph);
baseline_graph = true_graph;
odo_graph = true_graph;

for i = 1:D
   
    graphs = all_graphs{i};
    cg_err = zeros(1, numel(graphs));
    b_err = zeros(1, numel(graphs));
    o_err = zeros(1, numel(graphs));
    
    for j = 1:numel(graphs)
        
        cg = graphs(j);
        
        all_id = cg.robot_scope;
        zero_id = cg.base_id;
        zero_t = cg.time_scope(end);
        
        est = cg.subgraph.Zero(zero_id, zero_t);        
        est = est.GetSubset(all_id, zero_t);
        
        true = true_graph.GetSubset(all_id, zero_t);
        true = true.Zero(zero_id, zero_t);        
        
        baseline = baseline_graph.GetSubset(all_id, zero_t);
        baseline = baseline.Zero(zero_id, zero_t);
        
        odo = odo_graph.GetSubset(all_id, zero_t);
        odo = odo.Zero(zero_id, zero_t);                
                        
        cg_e = true.poses - est.poses;
        b_e = true.poses - baseline.poses;
        o_e = true.poses - odo.poses;
        
        cg_e(3,:) = wrapToPi(cg_e(3,:));
        b_e(3,:) = wrapToPi(b_e(3,:));
        o_e(3,:) = wrapToPi(o_e(3,:));
        
        cg_e(:,1) = [];
        b_e(:,1) = [];
        o_e(:,1) = [];
        
        cg_err(i) = mean(sqrt(sum(cg_e.*cg_e, 1)),2);
        b_err(i) = mean(sqrt(sum(b_e.*b_e, 1)),2);
        o_err(i) = mean(sqrt(sum(o_e.*o_e, 1)),2);
        
    end
    cg_errs(i) = mean(cg_err, 2);
    baseline_errs(i) = mean(b_err, 2);
    odo_errs(i) = mean(o_err, 2);
    
end

end