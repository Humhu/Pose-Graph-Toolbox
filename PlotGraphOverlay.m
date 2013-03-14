function PlotGraphOverlay(root, truth)

D = 2;
world_dims = [30,20];


belief_plotter = HierarchyPlotter(world_dims);
belief_plotter.z_scale = 0.5;
belief_plotter.colors = [0, 0, 0;
                         0, 0, 0;
                         0, 0, 0];
                     
truth_plotter = SequencePlotter(world_dims);
truth_plotter.z_scale = 0.5;
truth_plotter.colors = repmat([1,0,1],numel(truth(1).ids),1);
truth_plotter.Link(belief_plotter);

c = [0, 0, 1;
     0, 1, 0;
     1, 0, 0];
curr = root;                 

[tidMap, ttMap] = truth.BuildMaps();

belief_plotter.PlotLatestBeliefs(root);
truth_plotter.PlotSequence(truth(end));
truth_plotter.HideLines();
truth_plotter.HideLabels();
axis(belief_plotter.axe, 'tight');

for i = 1:D+1
       
    curr_cg = curr.chained_graph;
    gframe_curr_cg = curr_cg.ApplyLinks(curr_cg.depth+1);
    
    [idMap, tMap] = curr_cg.subgraph.BuildMaps();
    curr_idind = idMap.Forward(curr_cg.base_id);
    curr_tind = tMap.Forward(curr_cg.base_time);
    curr_bpose = gframe_curr_cg.subgraph(curr_tind).poses(:,curr_idind);
    
    t_idind = tidMap.Forward(curr_cg.base_id);
    t_tind = ttMap.Forward(curr_cg.base_time);
    t_bpose = truth(t_tind).poses(:,t_idind);
    
    dx = curr_bpose(1:2) - t_bpose(1:2);
    da = wrapToPi(curr_bpose(3) - t_bpose(3));
    shifted_truth = truth.Shift(-t_bpose(1:2));
    shifted_truth = shifted_truth.Rotate(da);  
    shifted_truth = shifted_truth.Shift(t_bpose(1:2));
    shifted_truth = shifted_truth.Shift(dx);
    
    truth_plotter = SequencePlotter(world_dims);
    truth_plotter.z_scale = 0.5;
    truth_plotter.colors = repmat(c(i,:),numel(truth(1).ids),1);
    truth_plotter.Link(belief_plotter);
    truth_plotter.PlotSequence(shifted_truth(end));    
    
    truth_plotter.HideLines();
    truth_plotter.HideLabels();
    
    axis(truth_plotter.axe, 'tight');
    
    if ~isempty(curr.followers)
        curr = curr.followers(1);
    end
    
end
