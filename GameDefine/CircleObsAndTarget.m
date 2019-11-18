%% ----------------------- Game define ----------------------------
% Problem Parameters.
%   targetRadius  Radius of target circle (positive).
%   targetCenter  center of target circle
%   velocityA	  Speed of the evader (positive constant).
%   velocityD	  Speed of the pursuer (positive constant).
%   obsCenter     Center of obs
%   obsRadius     Radius of obs
targetset.center = [-0,0];
targetset.radius = 0.1;
velocityA = gameParam.va; % Attacker
velocityD = gameParam.vd; % Defender
captureRadius = gameParam.captureRadius;

obs.center = [-0.35,0.05];
obs.radius = 0.08;
obs.type = 'circ';

dims_a = [1 1 0 0];
dims_d = [0 0 1 1];

%---------------------------------------------------------------------------
% TARGET SET
targetset.type = 'circ';

% target4D = shapeCylinder(g, [3 4], targetset.center, targetset.radius);
target4D = LevelsetFun(g, targetset, captureRadius);

% 2D version of grid
N2D = 400;
g2D = proj2D(g, target4D, dims_a, N2D);
target2D = createShape2D(g2D,targetset);
%---------------------------------------------------------------------------
% DOMAIN
[dom_bdry,dom_map] = std_domain(g2D);

%---------------------------------------------------------------------------
% OBSTACLES
% 4D Obstacles
obs_a = ObsFunction(g, obs, targetset, "attacker");
obs_d = ObsFunction(g, obs, targetset, "defender");

% 2D Obstacles
obs2D = createShape2D(g2D, obs);

%---------------------------------------------------------------------------
% PLAYER PARAMETERS
for i = 1:length(gameParam.xa_init)
    xa_init{i} = gameParam.xa_init{i};
    xd_init{i} = gameParam.xd_init{i};
% xa_init{2} = gameParam.xa_init{2};
% xd_init{2} = gameParam.xd_init{2};
end

%----------------------------------------------------------------------------
% TERMINAL AND AVOID SETS FOR 4D HJI COMPUTATION
attackerWin = target4D;

% - Defender has not captured the attacker
% Create capture condition, this is also part of the avoid set
% - The game type define the behavior of defenders
if gameParam.type == "chase"
    defenderWin = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
    defenderWin = sqrt(defenderWin) - captureRadius;
elseif gameParam.type == "protect"
    defenderWin = atan2(g.xs{2},g.xs{1}) - atan2(g.xs{4},g.xs{3});
    defenderWin = (abs(defenderWin)) - 0.08;
elseif gameParam.type == "both"
    defenderWin1 = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
    defenderWin1 = sqrt(defenderWin1) - captureRadius;
    defenderWin2 = atan2(g.xs{2},g.xs{1}).^2 - atan2(g.xs{4},g.xs{3}).^2;
    defenderWin2 = sqrt(abs(defenderWin2)) - 0.08;
    defenderWin = min(defenderWin1, defenderWin2);
end

% Mask the defender Win region from attacker Win region
attackerWin = shapeDifference(attackerWin, defenderWin);

%% ------------------- Utils Function ----------------------------
function data = LevelsetFun(g, t, captureRadius)
    xa = g.xs{1};
    ya = g.xs{2};
    xd = g.xs{3};
    yd = g.xs{4};
    
    % case1: attacker in target area
%     Target_a = sqrt((xa-t.center(1)).^2 + (ya-t.center(2)).^2) - t.radius;
      Target_a = shapeCylinder(g, [3 4], [0,0,t.center], t.radius);
    
    % case2: Not caught by defender
%     Catch = -((xd-xa).^2 + (yd-ya).^2 - captureRadius^2);
    
    % case3: defender not in target area
%     Target_d = -((xd-t.center(1)).^2 + (yd-t.center(2)).^2 - t.radius^2);
    
%     % case4: defender in obstacle
%     Obs_d = ((xd-o.Center(1)).^2 + (yd-o.Center(2)).^2 - o.Radius^2);

    % Terminal set: case1 && case2 && case3
%     data = (max(max(Target_a, Catch), Target_d));
%     data = (max(Target_a, Catch));
%     data = max(Target_a, Target_d);
    data = Target_a;
end

function obs = ObsFunction(g, o, t, type)
    xa = g.xs{1};
    ya = g.xs{2};
    xd = g.xs{3};
    yd = g.xs{4};
    
    % assign obstacle according to attacker and defender
    for i = 1:size(o.center, 1)
        if type == "defender"
            % obstacle 
%             obs1 = sqrt((xd-o.center(i,1)).^2 + (yd-o.center(i,2)).^2) - o.radius(i);
            obs1 = shapeCylinder(g, [1 2], [0,0,o.center], o.radius);
            % target set obs
%             obs2 = sqrt((xd-t.center(i,1)).^2 + (yd-t.center(i,2)).^2) - t.radius(i);
            obs2 = shapeCylinder(g, [1 2], [0,0,t.center], t.radius-0.06);
            
%             obs = obs1;
            obs = min(obs1, obs2);
        elseif type == "attacker"
%             obs = sqrt((xa-o.center(i,1)).^2 + (ya-o.center(i,2)).^2) - o.radius(i);
            obs = shapeCylinder(g, [3 4], [o.center], o.radius);
        end
    end
end
