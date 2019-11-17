function [ data, g, dataset, figStruct] = computeReachableSet(gameParam, accuracy, game, extraAvoid, extraAvoid_g)
% air3D: demonstrate the 3D aircraft collision avoidance example
%
%   [ data, g, data0 ] = air3D(accuracy)
%  
% In this example, the target set is a circle at the origin (cylinder in 3D)
% that represents a collision in relative coordinates between the evader
% (player a, fixed at the origin facing right) and the pursuer (player b).
%
% The relative coordinate dynamics are
%
%   \dot x    = -v_a + v_b \cos \psi + a y
%	  \dot y    = v_b \sin \psi - a x
%	  \dot \psi = b - a
%
% where v_a and v_b are constants, input a is trying to avoid the target
%	input b is trying to hit the target.
%
% For more details, see my F thesis, section 3.1.
%
% This function was originally designed as a script file, so most of the
% options can only be modified in the file.  For example, edit the file to
% change the grid dimension, boundary conditions, aircraft parameters, etc.
%
% To get exactly the result from the thesis choose:
%   targetRadius = 5, velocityA = velocityB = 5, inputA = inputB = +1.
%
% Input Parameters:
%
%   accuracy: Controls the order of approximations.
%     'low': Use odeCFL1 and upwindFirstFirst.
%     'medium': Use odeCFL2 and upwindFirstENO2 (default).
%     'high': Use odeCFL3 and upwindFirstENO3.
%     'veryHigh': Use odeCFL3 and upwindFirstWENO5.
%
% Output Parameters:
%
%   data: Implicit surface function at t_max.
%
%   g: Grid structure on which data was computed.
%
%   data0: Implicit surface function at t_0.

% Copyright 2004 Ian M. Mitchell (mitchell@cs.ubc.ca).
% This software is used, copied and distributed under the licensing 
%   agreement contained in the file LICENSE in the top directory of 
%   the distribution.
%
% Ian Mitchell, 3/26/04
% Subversion tags for version control purposes.
% $Date: 2012-07-04 14:27:00 -0700 (Wed, 04 Jul 2012) $
% $Id: air3D.m 74 2012-07-04 21:27:00Z mitchell $

%---------------------------------------------------------------------------
% You will see many executable lines that are commented out.
%   These are included to show some of the options available; modify
%   the commenting to modify the behavior.

%---------------------------------------------------------------------------
% Default parameters
if(nargin < 1), accuracy = 'low'; end
% if(nargin < 1), accuracy = 'medium'; end
% if(nargin < 1), accuracy = 'veryHigh'; end

% if nargin<2, game = 'midTarget_SimpleObs_fastD'; end
% if nargin<2, game = 'midTarget_LObs_vJ'; end
if nargin<2, game = 'OLGameModified'; end
% if nargin<2, game = 'LTarget_noObs'; end

if nargin<3, extraAvoid = []; extraAvoid_g = []; end

%---------------------------------------------------------------------------
% Integration parameters.
tMax = gameParam.timeMax;                  % End time.
plotSteps = 59;               % How many intermediate plots to produce?
t0 = 0;                      % Start time.
singleStep = 0;              % Plot at each timestep (overrides tPlot).
dt = 0.05;
tau = linspace(t0, tMax, plotSteps);

% Period at which intermediate plots should be produced.
tPlot = (tMax - t0) / (plotSteps - 1);

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

%---------------------------------------------------------------------------
% What level set should we view?
level = 0;

% Pause after each plot?
pauseAfterPlot = 0;

% Delete previous plot before showing next?
deleteLastPlot = 1;

% Visualize the angular dimension a little bigger.
aspectRatio = [ 1 1 0.4 ];

% Plot in separate subplots (set deleteLastPlot = 0 in this case)?
useSubplots = 0;

% Which of the tasks do you wish to perform?
doMask = 1;
doMin = 1;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = gameParam.Nx;

% Create the grid.
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [  1; 1; 1; 1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate;  @addGhostExtrapolate;  @addGhostExtrapolate };
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx; Nx ];
% Need to trim max bound in \psi (since the BC are periodic in this dimension).
g = processGrid(g);

%---------------------------------------------------------------------------
% Create initial conditions (cylinder centered on origin).
run(game);

% Initial conditions
data = shapeUnion(attackerWin,obs_d);

% The moving set can be anywhere outside the masked region.
%   So what we really need is the complement of the masked region.
% avoid = defenderWin;
avoid = shapeUnion(defenderWin,obs_a);

if(all(data(:) < 0) || (all(data(:) > 0)))
  warning([ 'Implicit surface not visible because function has ' ...
            'single sign on grid' ]);
end

data0 = data;

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @RAHamFunc;
schemeData.partialFunc = @RAPartialFunc;
schemeData.grid = g;

% The Hamiltonian and partial functions need problem parameters.
schemeData.velocityA = velocityA;
schemeData.velocityD = velocityD;

%---------------------------------------------------------------------------
% Choose degree of dissipation.

switch(dissType)
 case 'global'
  schemeData.dissFunc = @artificialDissipationGLF;
 case 'local'
  schemeData.dissFunc = @artificialDissipationLLF;
 case 'locallocal'
  schemeData.dissFunc = @artificialDissipationLLLF;
 otherwise
  error('Unknown dissipation function %s', dissFunc);
end

%---------------------------------------------------------------------------
if(nargin < 1)
  accuracy = 'medium';
end

% Set up time approximation scheme.
integratorOptions = odeCFLset('factorCFL', 0.9, 'stats', 'on');

% Choose approximations at appropriate level of accuracy.
switch(accuracy)
 case 'low'
  schemeData.derivFunc = @upwindFirstFirst;
  integratorFunc = @odeCFL1;
 case 'medium'
  schemeData.derivFunc = @upwindFirstENO2;
  integratorFunc = @odeCFL2;
 case 'high'
  schemeData.derivFunc = @upwindFirstENO3;
  integratorFunc = @odeCFL3;
 case 'veryHigh'
  schemeData.derivFunc = @upwindFirstWENO5;
  integratorFunc = @odeCFL3;
 otherwise
  error('Unknown accuracy level %s', accuracy);
end

if(singleStep)
  integratorOptions = odeCFLset(integratorOptions, 'singleStep', 'on');
end

%---------------------------------------------------------------------------
% Restrict the Hamiltonian so that reachable set only grows.
%   The Lax-Friedrichs approximation scheme MUST already be completely set up.
innerFunc = schemeFunc;
innerData = schemeData;
clear schemeFunc schemeData;

% Wrap the true Hamiltonian inside the term approximation restriction routine.
schemeFunc = @termRestrictUpdate;
schemeData.innerFunc = innerFunc;
schemeData.innerData = innerData;
schemeData.positive = 0;

% Set up masking so that the reachable set does not propagate through the
% avoid set
schemeData.maskData = -avoid(:);
schemeData.maskFunc = @max;


% Test mask
% schemeData.innerData.mask = -avoid(:);
% schemeData.innerData.doMask = doMask;
% 
% schemeData.innerData.min = data(:);
% schemeData.innerData.doMin = doMin;
% 
% integratorOptions = odeCFLset(integratorOptions, ...
%                               'postTimestep', @maskAndKeepMin);
% Let the integrator know what function to call.
integratorOptions = odeCFLset(integratorOptions, 'postTimestep', @postTimestepMask);
                     

%---------------------------------------------------------------------------
% Initialize Display
f1 = figure;
f2 = figure;

% Set up subplot parameters if necessary.
if(useSubplots)
  rows = ceil(sqrt(plotSteps));
  cols = ceil(plotSteps / rows);
  plotNum = 1;
  subplot(rows, cols, plotNum);
end

% h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(t0) ]);
hs = plotReachSets(g,g2D,target2D,obs2D,data,...
    xa_init,xd_init,captureRadius,f1,f2, dims_a, dims_d, dom_map);

camlight right;  camlight left;
hold on;
axis(g.axis);
daspect(aspectRatio);
drawnow;

%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
startTime = cputime;
i=1;

% -------------------------------------------------------------------------
% initiate dataset to restore Level set
clns = repmat({':'}, 1, g.dim);
data0size = size(data0);
dataset = zeros([data0size(1:g.dim) length(tau)]);
dataset(clns{:}, 1) = data0;


% -------------------------------------------------------------------------
while(tMax - tNow > small * tMax)

  % Reshape data array into column vector for ode solver call.
  y0 = data(:);

  % How far to step?
  tSpan = [ tNow, min(tMax, tNow + tPlot) ];
  
%   % Take a timestep.
%   [ t y ] = feval(integratorFunc, schemeFunc, tSpan, y0,...
%                   integratorOptions, schemeData);
              
  % Take a timestep.
  %   Record returned schemeData structure to keep track of min over time.
  [ t y schemeData ] = feval(integratorFunc, schemeFunc, tSpan, y0,...
                             integratorOptions, schemeData);
  tNow = t(end);

  % Get back the correctly shaped data array
  data = reshape(y, g.shape);
%   dataset(:,:,:,:,i) = data;
  dataset(clns{:}, i) = data;

  if(pauseAfterPlot)
    % Wait for last plot to be digested.
%     pause;
  end

  % Get correct figure, and remember its current view.
  figure(f1);
  [ view_az, view_el ] = view;

  % Delete last visualization if necessary.
    if deleteLastPlot
        [hs, rs2D, g2] = plotReachSets(g,g2D,target2D,obs2D,data,...
            xa_init,xd_init,captureRadius,f1,f2, dims_a, dims_d, dom_map,hs);
%          h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(tNow) ]);
    else
        hs = plotReachSets(g,g2D,target2D,obs2D,data,...
            xa_init,xd_init,captureRadius,f1,f2, dims_a, dims_d, dom_map);
%          h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(tNow) ]);
    end

  % Move to next subplot if necessary.
  if(useSubplots)
    plotNum = plotNum + 1;
    subplot(rows, cols, plotNum);
  end

  % Create new visualization.
%   plotReachSets(g,g2D,target2D,obs2D,data,...
%         xa_init,xd_init,captureRadius,f1,f2, dims_a, dims_d, dom_map)

  % Restore view.
  view(view_az, view_el);
  i=i+1;
end

% -------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% return struct needed for plot

figStruct = v2struct(tau, g,g2D,target2D,obs2D,...
                     xa_init,xd_init,captureRadius,...
                     dims_a, dims_d, dom_map, hs, rs2D, g2);

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);
end



%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = RAHamFunc(t, data, deriv, schemeData)
% air3DHamFunc: analytic Hamiltonian for 3D collision avoidance example.
%
% hamValue = air3DHamFunc(t, data, deriv, schemeData)
%
% This function implements the hamFunc prototype for the three dimensional
%   aircraft collision avoidance example (also called the game of
%   two identical vehicles).
%
% It calculates the analytic Hamiltonian for such a flow field.
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   deriv	 Cell vector of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%
%   hamValue	 The analytic hamiltonian.
%
% schemeData is a structure containing data specific to this Hamiltonian
%   For this function it contains the field(s):
%
%   .grid	 Grid structure.
%   .velocityA	 Speed of the evader (positive constant).
%   .velocityB	 Speed of the pursuer (positive constant).
%   .inputA	 Maximum turn rate of the evader (positive).
%   .inputB	 Maximum turn rate of the pursuer (positive).
%
% Ian Mitchell 3/26/04

grid = schemeData.grid;

% hamValue = (-(-schemeData.velocityA*sqrt(deriv{2}.^2+deriv{1}.^2)...
%             +schemeData.velocityD*sqrt(deriv{3}.^2+deriv{4}.^2)));

vd = schemeData.velocityD; % Defender velocity
va = schemeData.velocityA; % Attacker velocity

hamValue = -( - va*sqrt(deriv{1}.^2 + deriv{2}.^2) ...
    + vd*sqrt(deriv{3}.^2 + deriv{4}.^2)   );     
end



%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function alpha = RAPartialFunc(t, data, derivMin, derivMax, schemeData, dim)
% air3DPartialFunc: Hamiltonian partial fcn for 3D collision avoidance example.
%
% alpha = air3DPartialFunc(t, data, derivMin, derivMax, schemeData, dim)
%
% This function implements the partialFunc prototype for the three dimensional
%   aircraft collision avoidance example (also called the game of
%   two identical vehicles).
%
% It calculates the extrema of the absolute value of the partials of the 
%   analytic Hamiltonian with respect to the costate (gradient).
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   derivMin	 Cell vector of minimum values of the costate (\grad \phi).
%   derivMax	 Cell vector of maximum values of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%   dim          Dimension in which the partial derivatives is taken.
%
%   alpha	 Maximum absolute value of the partial of the Hamiltonian
%		   with respect to the costate in dimension dim for the 
%                  specified range of costate values (O&F equation 5.12).
%		   Note that alpha can (and should) be evaluated separately
%		   at each node of the grid.
%
% schemeData is a structure containing data specific to this Hamiltonian
%   For this function it contains the field(s):
%
%   .grid	 Grid structure.
%   .velocityA	 Speed of the evader (positive constant).
%   .velocityB	 Speed of the pursuer (positive constant).
%   .inputA	 Maximum turn rate of the evader (positive).
%   .inputB	 Maximum turn rate of the pursuer (positive).
%
% Ian Mitchell 3/26/04

grid = schemeData.grid;

vd = schemeData.velocityD;
va = schemeData.velocityA;

% Speeds of players
norm_va = sqrt(derivMax{1}.^2 + derivMax{2}.^2); % attacker
norm_vb = sqrt(derivMax{3}.^2 + derivMax{4}.^2); % defender
switch dim
    case 1
        alpha = va*abs(derivMax{1})./(norm_va);
%           alpha = va/2;
    case 2
        alpha = va*abs(derivMax{2})./(norm_va);
%           alpha = va/2;
    case 3
        alpha = vd*abs(derivMax{3})./(norm_vb);
%           alpha = vd/2;
    case 4
        alpha = vd*abs(derivMax{4})./(norm_vb);
%           alpha = vd/2;
  otherwise
    error([ 'Partials for the game of two identical vehicles' ...
            ' only exist in dimensions 1-3' ]);
end
end
