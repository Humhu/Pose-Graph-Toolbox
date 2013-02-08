% Randomly generates clusters of positions
%
% Inputs:
%   centers - 3 X N array of cluster centers
%   sizes - 1 X N array of cluster point quantities
%   covariances - 1 X N cell array of 3 X 3 cluster covariances
%   seed - Random number generator seed
%
% Outputs:
%   positions - 1 X N cell array. Each element is a 3 X ? array of
%   clustered points
function [positions] = GenerateClusters(centers, sizes, covariances, seed)

newStream = RandStream('mt19937ar', 'Seed', seed);

prevStream = RandStream.getGlobalStream();
RandStream.setGlobalStream(newStream);

positions = cell(1,numel(sizes));

for i = 1:size(centers, 2)
   
    positions{i} = mvrnd(centers(:,i), covariances{i}, sizes(i))';
    
end

RandStream.setGlobalStream(prevStream);