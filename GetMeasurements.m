function [ranges, angles] = GetMeasurements(poses, covariance)

N = numel(poses);
relations = GetRelations(poses);
ranges = zeros(N,N);
angles(N,N) = Orientation1D;

for i = 1:N
    for j = 1:N
        noise = mvnrnd([0; 0], covariance);
        ranges(i,j) = norm(relations(i,j).position) + noise(1);
        angles(i,j) = relations(i,j).orientation + Orientation1D(noise(2));
    end
end

for i = 1:N
    ranges(i,i) = 0;
    angles(i,i) = Orientation1D(0);
end