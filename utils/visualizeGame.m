function [colors, hs] = visualizeGame(g2D, target2D, obs2D, xas, xds, captureRadius, dom_map, hs)
% [colors, hs] = visualizeGame(g2D, target2D, obs2D, xas, xds, captureRadius, dom_map, hs)

if nargin<7, [~, dom_map] = std_domain(g2D); end

% if exist('hs','var')
%     delete(hs.ht)
%     delete(hs.ho)
%     for i = 1:length(hs.hxas), delete(hs.hxas{i}); end
%     for i = 1:length(hs.hxds), delete(hs.hxds{i}); end
%     for i = 1:length(hs.hxdcaps), delete(hs.hxdcaps{i}); end
% end

targetColor = [0 0.5 0];
obsColor = 'k';
attackerColor = [0.75 0 0];
defenderColor = 'b';

colors = v2struct(targetColor, obsColor, attackerColor, defenderColor);

% TARGET
[Ct, ht] = contour(g2D.xs{1}, g2D.xs{2}, target2D, [0 0], ...
    'color', 'b', 'linewidth',2);
hold on

% OBSTACLES
[Co, ho] = contour(g2D.xs{1}, g2D.xs{2}, obs2D, [0 0], ...
    'color', obsColor, 'linewidth',2);

[Cd, hd] = contour(g2D.xs{1}, g2D.xs{2}, dom_map, [0 0], ...
    'color', obsColor, 'linewidth',2);

% ATTACKERS
hxas = cell(length(xas),1);
if ~isempty(xas)
    if ~iscell(xas), xas = {xas}; end
    hxas = cell(length(xas),1);
    for i = 1:length(xas)
        hxas{i} = plot(xas{i}(1), xas{i}(2), 'o',...
            'color','r');
    end
end

% DEFENDERS
hxds = cell(length(xds),1);

xdcap = cell(length(xds),1);
hxdcaps = cell(length(xds),1);
Cxdcaps = cell(length(xds),1);

if ~isempty(xds)
    if ~iscell(xds), xds = {xds}; end
    for i = 1:length(xds)
        hxds{i} = plot(xds{i}(1), xds{i}(2), '*',...
            'color','b');

    %     xdcap{i} = captureSet(g2D,xds{i},captureRadius);
    %     [Cxdcaps{i}, hxdcaps{i}] = contour(g2D.xs{1}, g2D.xs{2}, xdcap{i}, [0 0], ...
    %         'color',defenderColor);
    end
end

axis(g2D.axis)
axis square

hs = v2struct(ht, ho, hxas, hxds, hxdcaps);
end

function s = v2struct(varargin)
for i = 1:nargin
    s.(inputname(i))=varargin(i);
end
end