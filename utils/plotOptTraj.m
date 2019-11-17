function plotOptTraj(s, traj, tau)
% Plots the 2D reachable sets in a multiplayer reach avoid game
%
% Inputs
%   s:    struct contains the variables to construct the simulation
%         environment
%   traj: the waypoints needed for spline
%   tau:  the time steps from reachable set computation
%
%   Yuhan Peng, 2019-11-10

%% ----------------- Initialize parameters ------------------------
% extract all variables from struct
g2D = s.g2D{1};
target2D = s.target2D{1};
obs2D = s.obs2D{1};
xas = s.xa_init{1};
xds = s.xd_init{1};
captureRadius = s.captureRadius{1};
dom_map = s.dom_map{1};
data_slice_d = s.rs2D{1};
g2 = s.g2{1};

% define level set
level = 0;

%% --------------- Plot defender fixed reachable set ---------------
rs = figure();
figure(rs);
reachableSet = data_slice_d{1};
for i = 1:size(xds,2) % Fix defender position
    
    % Plot game setup
    if exist('hs','var') % If we want to delete last plot
        [~, hsd] = visualizeGame(g2D,target2D,obs2D,xas{i},xds{i}, captureRadius, dom_map,hs);
    else
        [~, hsd] = visualizeGame(g2D,target2D,obs2D,xas,xds, captureRadius, dom_map);
    end
    
%     hd{i} = visualizeLevelSet(g2, data_slice_d{i}, displayType, level);
    reachableSet = max(reachableSet, data_slice_d{i});
    hold on;
    axis(g2.axis);
    drawnow;
end
    % Plot reachable set
    [~, hd] = contour(g2.xs{1}, g2.xs{2}, reachableSet, [level level], 'linecolor', 'r');
%% ------------------ Plot players trajectory ----------------------
% tau and traj initialize
if tau(1) ~= 0
   tau = [0, tau]; 
end

for i = 1:size(xas,2)
    traj{i} = [[xas{i}';xds{i}'],traj{i}];
    traj_size(1,i) = size(traj{i},2); 
end

% fill up the trajectory
[val, idx] = max(traj_size);
for i = 1:size(xas,2)
   if i~=idx
      traj{i} = [traj{i},traj{i}(:, end).* ones(4,val-traj_size(i))]; 
   end
end

% spline trajectory 
xa_spline = cell(size(xas,2));
for i = 1:size(xas,2)
    xa_spline{i} = spline(tau, traj{i}(1:2,:));
    xd_spline{i} = spline(tau, traj{i}(3:4,:));
end

% trajectory simulation
% - initiate parameter
t0 = 0;
t = t0;
figure(rs);
xa_old = xas;
xd_old = xds;
xa_now = cell(size(xas));
xd_now = cell(size(xas));

% save video
pic_num =1;

% compute all waypoints
t = 0:0.005:tau(end);
for i = 1:size(xas,2)
       % evaluate value from spline
       xa_now{i} = ppval(xa_spline{i}, t);
       xd_now{i} = ppval(xd_spline{i}, t);
end

% - main loop
for k = 2:size(t,2)
    for i = 1:size(xas,2)       
       % update figure
       set(hsd.hxas{1}{i}, 'XData', xa_now{i}(1,k), 'YData', xa_now{i}(2,k));
       set(hsd.hxds{1}{i}, 'XData', xd_now{i}(1,k), 'YData', xd_now{i}(2,k));
       
       % add trajectory
       line([xa_now{i}(1,k); xa_now{i}(1,k-1)], [xa_now{i}(2,k); xa_now{i}(2,k-1)], 'color', [1,0,0], 'lineWidth', 2);
       line([xd_now{i}(1,k); xd_now{i}(1,k-1)], [xd_now{i}(2,k); xd_now{i}(2,k-1)], 'color', [0,0,0], 'lineWidth', 2);
    end
    drawnow;
    
    % save gif
    F=getframe(rs);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map,'test.gif','gif', 'DelayTime',0);
    else
        imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0);
    end
    pic_num = pic_num + 1;

end