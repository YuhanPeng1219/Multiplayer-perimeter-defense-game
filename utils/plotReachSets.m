function [hs, data_slice_d, g2] = plotReachSets(g, g2D, target2D, obs2D, data,...
    xas, xds, captureRadius, f1, f2, dims_a, dims_d, dom_map, hs)
% plotReachSets(g,g2D,target2D,obs2D,data,...
%    xas,xds,captureRadius,f1,f2, dims_a, dims_d, dom_map)
% Plots the 4D reachable sets in a multiplayer reach avoid game
%
% Inputs
%   g:          4D grid structure
%   g2D:        2D grid structure
%   target2D:   2D representation of the target set
%   obs2D:        2D representation of the obstacles
%   data:       4D HJI solution to the 1 vs. 1 problem
%   xas:        cell structure of positions of all attackers
%   xds:        cell structure of positions of all defenders
%   captureRadius:  attacker is captured if he gets within this distance to
%                   the defender
%   f1, f2:     figure handles
%   dims_a:     dimensions of the attacker ([1 1 0 0], usually)
%   dims_d:     dimensions of the attacker ([1 1 0 0], usually)
%
% Mo Chen, 2013-11-21

level = 0;

N2D = 200;

spRd = ceil( sqrt(size(xds,1)) );
spCd = ceil( size(xds,1)/spRd );
spRa = ceil( sqrt(size(xas,1)) );
spCa = ceil( size(xas,1)/spRa );


data_slice_d = cell(size(xds,1),1);
data_slice_a = cell(size(xas,1),1);

if exist('hs','var') % If we want to delete last plot
    for i = 1:size(xds,1), delete(hs.hd{i}); end
%     for i = 1:size(xas,1), delete(hs.ha{i}); end
else
    hd = cell(size(xds,1),1);
%     ha = cell(size(xas,1),1);
end

figure(f1)
for i = 1:size(xds,2) % Fix defender position
    subplot(2,1,i)
    
    % Plot game setup
    if exist('hs','var') % If we want to delete last plot
        [~, hsd] = visualizeGame(g2D,target2D,obs2D,xas{i},xds{i}, captureRadius, dom_map,hs);
    else
        [~, hsd] = visualizeGame(g2D,target2D,obs2D,xas{i},xds{i}, captureRadius, dom_map);
    end

    % Plot reachable set
    [g2, data_slice_d{i}] = proj2D(g, data, dims_d, xds{i}, N2D);  
    if i == 1
        [~, hd{i}] = contour(g2.xs{1}, g2.xs{2}, data_slice_d{i}, [level level], 'linecolor', 'r');
    else
        [~, hd{i}] = contour(g2.xs{1}, g2.xs{2}, data_slice_d{i}, [level level], 'linecolor', 'r');
%     hd{i} = visualizeLevelSet(g2, data_slice_d{i}, displayType, level);
    end
    hold off;
    axis(g2.axis);
    drawnow;
end

% figure(f2)
% for i = 1:size(xas,1) % Fix attacker position
%     subplot(spRa,spCa,i)
%     
%     % Plot game setup
%     if exist('hs','var') % If we want to delete last plot
%         [colors, hsa] = visualizeGame(g2D, target2D, obs2D, xas{i}, xds, captureRadius, dom_map,hs);
%     else
%         [colors, hsa] = visualizeGame(g2D, target2D, obs2D, xas{i}, xds, captureRadius, dom_map);
%     end
%     
%     [g2, data_slice_a{i}] = proj2D(g, data, dims_a, xds{i}, N2D);
%     [~, ha{i}] = contour(g2.xs{1}, g2.xs{2}, data_slice_a{i}, [level level], 'linecolor', 'b');
% %     ha{i} = visualizeLevelSet(g2, data_slice_a{i}, displayType, level);
% %     set(ha{i},'color', colors.attackerColor)
% %     
% %     
% %     if exist('extraAvoid','var')
% %         if ~isempty(extraAvoid)
% %             [g2, data_slice] = proj2D(g, dims_d, N2D, extraAvoid, xds{1});
% %             oea = visualizeLevelSet(g2, data_slice, displayType, level);
% %             set(oea, 'color','k')
% %         end
% %     end
%     
%     hold on;
%     axis(g2.axis);
%     drawnow;
% end

hs = hsd;
% hs.hxds = hsa.hxds;
hs.hd = hd;
% hs.ha = ha;
return
%---------------------------------------------------------------------------
% Figure positioning (for the ACC 2014 paper)
% Subplot positions
% Subplot spacing!
subP_size = 0.325;
subP_xmin = 0.05;
subP_ymin = 0.1;
subP_xgap = 0.03;
subP_ygap = 0.15;

subP_pos = [subP_xmin               subP_ymin+subP_size+subP_ygap       subP_size subP_size;
    subP_xmin+subP_size+subP_xgap   subP_ymin+subP_size+subP_ygap   subP_size subP_size;
    subP_xmin                       subP_ymin                      subP_size subP_size;
    subP_xmin+subP_size+subP_xgap   subP_ymin                      subP_size subP_size];

figure(f1)
% for i = 1:size(xds,1) % Fix defender position
%     subplot(spRd,spCd,i)
%     set(gca,'position',subP_pos(i,:))
% end
pos = get(gcf,'position');
set(gcf,'position',[200 100 800 600]);

legend([hsd.ht hsd.ho hsd.hxds{1} hsd.hxas{1} hd{end}], ...
    {'Target','Obstacle','Defender','Attacker','Reach-Avoid Slice'})
set(legend,'units','pixels','position',[560 100 250 125])

figure(f2)
% for i = 1:size(xas,1) % Fix defender position
%     subplot(spRa,spCa,i)
%     set(gca,'position',subP_pos(i,:))
% end
pos = get(gcf,'position');
set(gcf,'position',[300 100 800 600]);

legend([hsa.ht hsa.ho hsa.hxds{1} hsa.hxas{1} ha{end}], ...
    {'Target','Obstacle','Defender','Attacker','Reach-Avoid Slice'})
set(legend,'units','pixels','position',[560 100 250 125])

hs = v2struct(hd, ha, hsa,hsd);
end
