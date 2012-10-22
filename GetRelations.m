function [relations] = GetRelations(poses)

N = numel(poses);
relations(N,N) = Pose2D;

for i = 1:N
    for j = 1:N
        rel_pos = poses(j).position - poses(i).position;
        rel_bearing = Orientation1D(atan2(rel_pos(2), rel_pos(1))) - poses(i).orientation;
        relations(i,j) = Pose2D(rel_pos, rel_bearing);
    end
end