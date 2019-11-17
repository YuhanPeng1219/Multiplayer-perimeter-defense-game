function [ data, g, data0 ] = air3D(accuracy)
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
% Make sure we can see the kernel m-files.
run('addPathToKernel.m');

close all;
%---------------------------------------------------------------------------
% Integration parameters.
tMax = 5.8;                  % End time.
plotSteps = 9;               % How many intermediate plots to produce?
t0 = 0;                      % Start time.
singleStep = 0;              % Plot at each timestep (overrides tPlot).

% Period at which intermediate plots should be produced.
tPlot = (tMax - t0) / (plotSteps - 1);

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

%---------------------------------------------------------------------------
% Problem Parameters.
%   targetRadius  Radius of target circle (positive).
%   velocityA	  Speed of the evader (positive constant).
%   velocityB	  Speed of the pursuer (positive constant).
%   inputA	  Maximum turn rate of the evader (positive).
%   inputB	  Maximum turn rate of the pursuer (positive).
targetRadius = 0.2;
velocityA = 0.1; % Attacker
velocityB = .1; % Defender
captureRadius = 0.05;
obsCenter = [0.5,0.5];
obsRadius = 0.15;

inputA = 1;
inputB = 1;

%---------------------------------------------------------------------------
% What level set should we view?
level = 0;

% Visualize the 3D reachable set.
displayType = 'surface';

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
Nx = 31;

% Create the grid.
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [  1; 1; 1; 1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate;  @addGhostExtrapolate;  @addGhostExtrapolate };
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; ceil(Nx * (g.max(2) - g.min(2)) / (g.max(1) - g.min(1))); Nx;  ceil(Nx * (g.max(4) - g.min(4)) / (g.max(3) - g.min(3)))];
% Need to trim max bound in \psi (since the BC are periodic in this dimension).
% g.max(3) = g.max(3) * (1 - 1 / g.N(3));
g = processGrid(g);

%---------------------------------------------------------------------------
% Create initial conditions (cylinder centered on origin).
% data = shapeCylinder(g, 3, [ 0; 0; 0 ], targetRadius);
data = LevelsetFun(g, targetRadius, captureRadius, obsCenter, obsRadius);
mask = MaskFun(g, targetRadius, captureRadius, obsCenter, obsRadius);

% The moving set can be anywhere outside the masked region.
%   So what we really need is the complement of the masked region.
mask = -mask;

% Need to ensure that the initial conditions satisfy the mask.
data = max(data, mask);

if(all(data(:) < 0) || (all(data(:) > 0)))
  warning([ 'Implicit surface not visible because function has ' ...
            'single sign on grid' ]);
end

data0 = data;

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @air3DHamFunc;
schemeData.partialFunc = @air3DPartialFunc;
schemeData.grid = g;

% The Hamiltonian and partial functions need problem parameters.
schemeData.velocityA = velocityA;
schemeData.velocityB = velocityB;
schemeData.inputA = inputA;
schemeData.inputB = inputB;

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
integratorOptions = odeCFLset('factorCFL', 0.75, 'stats', 'on');

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
% Set up data required for the mask operation.
%   Mask will be compared to vector form of data array used by integrator.
schemeData.mask = mask(:);
schemeData.doMask = doMask;

% Also keep track of minimum of phi over time.
%   Minimum will be kept in vector form used by integrator.
schemeData.min = data(:);
schemeData.doMin = doMin;

% Let the integrator know what function to call.
integratorOptions = odeCFLset(integratorOptions, ...
                              'postTimestep', @maskAndKeepMin);
                        
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

%---------------------------------------------------------------------------
% Initialize Display
f = figure;

% Set up subplot parameters if necessary.
if(useSubplots)
  rows = ceil(sqrt(plotSteps));
  cols = ceil(plotSteps / rows);
  plotNum = 1;
  subplot(rows, cols, plotNum);
end

h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(t0) ]);

camlight right;  camlight left;
hold on;
axis(g.axis);
daspect(aspectRatio);
drawnow;

%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
startTime = cputime;
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

  if(pauseAfterPlot)
    % Wait for last plot to be digested.
%     pause;
  end

  % Get correct figure, and remember its current view.
  figure(f);
  [ view_az, view_el ] = view;

  % Delete last visualization if necessary.
  if(deleteLastPlot)
    delete(h);
  end

  % Move to next subplot if necessary.
  if(useSubplots)
    plotNum = plotNum + 1;
    subplot(rows, cols, plotNum);
  end

  % Create new visualization.
  h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(tNow) ]);

  % Restore view.
  view(view_az, view_el);

%     ind = find(data<0);
%     [a,b,c,d] = ind2sub(size(data), ind);
%     c = -3+c*6/11;
%     d = -3+d*6/11;
%     k = boundary(c,d);
%     hold on;
%     plot(c(k),d(k));
  
end

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = air3DHamFunc(t, data, deriv, schemeData)
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

checkStructureFields(schemeData, 'grid', 'velocityA', 'velocityB', ...
                                 'inputA', 'inputB');

grid = schemeData.grid;

% implements equation (3.3) from my thesis term by term
%   with allowances for \script A and \script B \neq [ -1, +1 ]
%   where deriv{i} is p_i
%         x_r is grid.xs{1}, y_r is grid.xs{2}, \psi_r is grid.xs{3}
%         v_a is velocityA, v_b is velocityB, 
%         \script A is inputA and \script B is inputB

% hamValue = -abs(-schemeData.velocityA*sqrt(deriv{1}.^2+(deriv{3}.^2./(grid.xs{1}+1).^2))...
%             + schemeData.velocityB*sqrt(deriv{2}.^2+(deriv{3}.^2./(grid.xs{2}+1).^2)));
hamValue = (-(-schemeData.velocityA*sqrt(deriv{2}.^2+deriv{1}.^2)...
            +schemeData.velocityB*sqrt(deriv{3}.^2+deriv{4}.^2)));
     
end



%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function alpha = air3DPartialFunc(t, data, derivMin, derivMax, schemeData, dim)
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

checkStructureFields(schemeData, 'grid', 'velocityA', 'velocityB', ...
                                 'inputA', 'inputB');

grid = schemeData.grid;

switch dim
  case 1
    alpha = schemeData.velocityA;

  case 2
    alpha = schemeData.velocityA;

  case 3
    alpha = schemeData.velocityB;
    
   case 4
    alpha = schemeData.velocityB;

  otherwise
    error([ 'Partials for the game of two identical vehicles' ...
            ' only exist in dimensions 1-3' ]);
end
end

%% ***************** LEVEL SET FUNCTION ********************
function data = LevelsetFun(g, targetRadius, captureRadius, obsCenter, obsRadius)
    xa = g.xs{1};
    ya = g.xs{2};
    xd = g.xs{3};
    yd = g.xs{4};
    
    % case1: attacker in target area
    Target_a = xa.^2 + ya.^2 - targetRadius^2;
    
    % case2: Not caught by defender
    Catch = -((xd-xa).^2 + (yd-ya).^2 - captureRadius^2);
    
    % case3: defender not in target area
    Target_d = -(xd.^2 + yd.^2 - targetRadius^2);
    
    % case4: defender in obstacle
    Obs_d = ((xd-obsCenter(1)).^2 + (yd-obsCenter(2)).^2 - obsRadius^2);

    % Terminal set: case1 && case2 && case3
    data = min(max(max(Target_a, Catch), Target_d), Obs_d);
%     data = Target_a;
    
end

function mask = MaskFun(g, targetRadius, captureRadius, obsCenter, obsRadius)
    xa = g.xs{1};
    ya = g.xs{2};
    xd = g.xs{3};
    yd = g.xs{4};
    
    % case1: caught by defender
    Catch = ((xd-xa).^2 + (yd-ya).^2 - captureRadius^2);
    
    % case2: defender in obstacle
    Obs_a = ((xa-obsCenter(1)).^2 + (ya-obsCenter(2)).^2 - obsRadius^2);  
    
    % Terminal set: case1 && case2 && case3
    mask = min(Catch, Obs_a);
end

function [ yOut, schemeDataOut ] = maskAndKeepMin(~, yIn, schemeDataIn)
% maskAndKeepMin: Example postTimestep processing routine.
%
%  [ yOut, schemeDataOut ] = maskAndKeepMin(t, yIn, schemeDataIn)
%
%  This function demonstrates two entirely different processes that
%    can be accomplished through the postTimestep odeCFLn integrator option.
%
%  The first is to mask the evolving implicit surface function
%    (or otherwise modify its value after each timestep).
%
%  In this case masking is accomplished by
%
%          yOut = max(yIn, schemeDataIn.mask);
%
%
%   which ensures that phi cannot be negative anywhere that mask is positive.
%
%  The second is to keep track of some feature of that implicit surface
%    function or otherwise modify the schemeData structure after each
%    timestep.
%
%  In this case the feature recorded is the pointwise minimum over time of phi
%
%          schemeDataOut.min = min(yIn, schemeDataIn.min);
%
%
% Parameters:
%   t              Current time.
%   yIn            Input version of the level set function, in vector form.
%   schemeDataIn   Input version of a structure (see below).
%
%   yOut           Output version of the level set function, in vector form.
%   schemeDataOut  Output version of the structure (possibly modified).
%
% schemeData is a structure containing data specific to this type of 
%   term approximation.  For this function it contains the field(s)
%
%   .doMask      Boolean specifying whether masking should be performed.
%   .doMin       Boolean specifying whether min should be taken.
%   .mask	 Function against which to mask the level set function.
%   .min         Function which stores the minimum of the level set
%                  function over time (it is modified at each timestep).
%
% schemeData may contain other fields.
%   schemeDataIn = schemeDataIn.innerData;
  checkStructureFields(schemeDataIn.innerData, 'doMask', 'doMin');

  % Mask the current level set function.
  if(schemeDataIn.innerData.doMask)
    checkStructureFields(schemeDataIn.innerData, 'mask');
    yOut = max(yIn, schemeDataIn.innerData.mask);
  else
    yOut = yIn;
  end

  % Record any new minimum values for each node.
  %   Use yOut to get the masked version of the data (if masking).
  schemeDataOut = schemeDataIn;
  schemeDataOut.innerData = schemeDataIn.innerData;
  if(schemeDataIn.innerData.doMin)
    checkStructureFields(schemeDataIn.innerData, 'min');
    schemeDataOut.innerData.min = min(yOut, schemeDataOut.innerData.min);
  end
end
end
