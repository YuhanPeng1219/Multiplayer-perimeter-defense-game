function [traj, traj_tau] = OptTrajCal(g, data, tau, gameParam, extraArgs)
% [traj, traj_tau] = computeOptTraj(g, data, tau, dynSys, extraArgs)
%   Computes the optimal trajectories given the optimal value function
%   represented by (g, data), associated time stamps tau, dynamics given in
%   dynSys.
%
% Inputs:
%   g, data - grid and value function
%   tau     - time stamp (must be the same length as size of last dimension of
%                         data)
%   dynSys  - dynamical system object for which the optimal path is to be
%             computed
%   extraArgs
%     .uMode        - specifies whether the control u aims to minimize or
%                     maximize the value function
%     .visualize    - set to true to visualize results
%     .fig_num:   List if you want to plot on a specific figure number
%     .projDim      - set the dimensions that should be projected away when
%                     visualizing
%     .fig_filename - specifies the file name for saving the visualizations

if nargin < 5
  extraArgs = [];
end

% Default parameters
uMode = 'min';
visualize = false;
subSamples = 4;

if isfield(extraArgs, 'uMode')
  uMode = extraArgs.uMode;
end

% Visualization
if isfield(extraArgs, 'visualize') && extraArgs.visualize
  visualize = extraArgs.visualize;
  
  showDims = find(extraArgs.projDim);
  hideDims = ~extraArgs.projDim;
  
  if isfield(extraArgs,'fig_num')
    f = figure(extraArgs.fig_num);
  else
    f = figure;
  end
end

if isfield(extraArgs, 'subSamples')
  subSamples = extraArgs.subSamples;
end

clns = repmat({':'}, 1, g.dim);

tau = tau{1};
if any(diff(tau)) < 0
  error('Time stamps must be in ascending order!')
end

% Time parameters
iter = 1;
tauLength = length(tau);
dtSmall = (tau(2) - tau(1))/subSamples;
dt = tau(2)-tau(1);
% maxIter = 1.25*tauLength;

% Initialize trajectory
% traj = nan(g.dim, tauLength);
% traj(:,1) = dynSys.x;
tEarliest = ones(1, size(gameParam.xa_init,2));
att_caught = zeros(1, size(gameParam.xa_init,2));
att_reach = zeros(1, size(gameParam.xa_init,2));

% Initialize state
for i = 1:size(gameParam.xa_init,2)
    xs{i} = [gameParam.xa_init{i}';gameParam.xd_init{i}'];
%     xs{2} = [gameParam.xa_init{2}';gameParam.xd_init{2}'];
    traj{i} = xs{i};
end
traj_tau = tau(1);

while iter <= tauLength 
  % Determine the earliest time that the current state is in the reachable set
  % Binary search
  for i = 1:size(gameParam.xa_init,2)
      % determine if caught by defender
      if norm(traj{i}(1:2, end)-traj{i}(3:4, end)) < gameParam.captureRadius
         fprintf("Attacker(%d) is caught by defender \n", i);
         att_caught(i) = 1;
         continue
      end
      
      upper = tauLength;
      lower = tEarliest(i);

      tEarliest(i) = find_earliest_BRS_ind(g, data, xs{i}, upper, lower);
      
%       if i==2 && iter==1
%          tEarliest(i) = 20; 
%       end

      % BRS at current time
      BRS_at_t = data(clns{:}, tEarliest(i));

      if tEarliest(i) == tauLength
        % Trajectory has entered the target
        fprintf("Attacker(%d) is in target set \n", i);
        att_reach(i) = 1;
        continue
      end

      Deriv = computeGradients(g, BRS_at_t);
      deriv = eval_u(g, Deriv, xs{i});

      dira = -deriv(1:2)/norm(deriv(1:2));
      dird = deriv(3:4)/norm(deriv(3:4));

      xs{i}(1:2) = xs{i}(1:2) + dira*gameParam.va*dt;
      xs{i}(3:4) = xs{i}(3:4) + dird*gameParam.vd*dt;
      
      x_temp = xs{i};
      x_temp(1:2) = x_temp(1:2)/norm(x_temp(1:2));
      x_temp(3:4) = x_temp(3:4)/norm(x_temp(3:4));
      tan_difference = atan2(x_temp(2),x_temp(1)) - atan2(x_temp(4),x_temp(3));
      fprintf("tan diff of pair(%d) is %d \n", i, tan_difference);

%       xs{i} = [xa;xd];

      hold on 
      scatter(xs{i}(1),xs{i}(2), 'r');
      scatter(xs{i}(3),xs{i}(4), 'b');
      drawnow;

      traj{i} = [traj{i},xs{i}];
  end
  iter = iter + 1;
  if iter>59
     break 
  end
  traj_tau = [traj_tau, tau(iter)];
  if all(any([att_caught;att_reach])) == 1
      break
  end
  
%   if all(att_reach) == 1
%       break
%   end
end