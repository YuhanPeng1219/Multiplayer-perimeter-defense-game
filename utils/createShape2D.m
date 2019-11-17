function shape = createShape2D(g, shape, clrc)
% function obs = createObstacles4D(g, obs, obs_type)
% Creates an 2D obstacle based on fields in obs
%
% Inputs:
%   g           - grid structure
%   shape       - shape structure
%   clrc        - enlarge the shape by this amount in all dimensions
%
% Outputs:
%   obs         - level set representation of the obstacle (<0 means
%                   inside)
%
% Mo Chen, Sept. 26, 2013
%

if nargin<3, clrc = 0; end

switch shape.type
    case 'rect'
        % Obstacle in attacker space
        shape = shapeRectangleByCorners(g, ...
            [shape.xmin-clrc shape.ymin-clrc], ...
            [shape.xmax+clrc shape.ymax+clrc]);
        
    case 'circ'
        shape = shapeCylinder(g, [], shape.center, shape.radius+clrc);
        
    otherwise
        error('Unknown obstacle type!')
end
end
