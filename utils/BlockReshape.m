% Only intended to work for 2D matrices
% TODO: Finish!
function [Mr] = BlockReshape(M, bsize, rsize)

bheight = bsize(1);
bwidth = bsize(2);
num_blocks = size(M,1)*size(M,2)/(bheight*bwidth);

pD = reshape(M, bheight, bwidth, num_blocks);
pD = permute(pD, [2,1,3]);
pD = reshape(pD, bwidth, *bheight);
pD = permute(pD, [2,1]);