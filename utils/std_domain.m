function [dom_bdry,dom_map] = std_domain(g)
% Standard square domain
% Set up figure based on grid
%
% INPUT
%   grid:
% OUTPUT
%   dom_bdry:   list of indices of boundary points
%   dom_map:    N by N map of domain (-1 means outisde, 1 means inside)
%
% Mo Chen, 2013-06-07
%

% checkGrid(g);

N = g.N(1);
X = g.vs{1}';
Y = g.vs{2}';

% construct list of boundary indices, go clock-wise
bot =   [X(1:N-1)'          ones(N-1,1)*Y(1)];  % fix y min, increase in x
right = [ones(N-1,1)*X(N)   Y(1:N-1)'       ];  % fix x max, increase in y
top =   [X(N:-1:2)'         ones(N-1,1)*Y(N)];  % fix y max, decrease in x
left =  [ones(N-1,1)*X(1)   Y(N:-1:2)'];  % fix x min, decrease in y

x = [bot(:,1); right(:,1); top(:,1); left(:,1)];
y = [bot(:,2); right(:,2); top(:,2); left(:,2)];

% Convert to grid indices
[ix, iy] = xy2inds(x,y,g);
dom_bdry = [ix iy];

% Determine which grid points are inside and outside
[iX, iY] = ndgrid(1:N,1:N);
dom_map = single(inpolygon(iX,iY,ix,iy));
dom_map(dom_map==0)=-1;
dom_map = -dom_map; % By level set convention, negative corresponds to inside

end

function [ix, iy] = xy2inds(x,y,g)
min = g.min;
max = g.max;

N = g.N(1);

ix = [1:N,(N+1)*ones(1,N),(N+1):-1:2,ones(1,N)]';
iy = [ones(1,N),1:N,(N+1)*ones(1,N),N+1:-1:2]';
end