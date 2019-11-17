%---------------------------------------------------------------------------
% DYNAMICS
captureRadius = 0.1;
velocitya = 1;
velocityd = 1;

dims_a = [1 1 0 0];
dims_d = [0 0 1 1];

%---------------------------------------------------------------------------
% TARGET SET
targetset.center = 0.1;
targetset.radius = 0.1;
targetset.type = 'circ';

target4D = shapeCylinder(g, [3 4], targetset.center, targetset.radius);
% target4D = LevelsetFun(g, 0.1);

% 2D version of grid
N2D = 200;
g2D = proj2D(g, target4D, dims_a, N2D);
target2D = createShape2D(g2D,targetset);
%---------------------------------------------------------------------------
% DOMAIN
[dom_bdry,dom_map] = std_domain(g2D);

%---------------------------------------------------------------------------
% OBSTACLES
% (No obstacles)
obs2D = ones(g2D.N');               % 2D obstacles
obs_a = ones(g.N');     % 4D obstacles for attacker
obs_d = ones(g.N');     % 4D obstacles for defender

%---------------------------------------------------------------------------
% PLAYER PARAMETERS
xa_init{1} = [-0.5 -0.6];
xd_init{1} = [0.1 0];

%----------------------------------------------------------------------------
% TERMINAL AND AVOID SETS FOR 4D HJI COMPUTATION
attackerWin = target4D;

% - Defender has not captured the attacker
% Create capture condition, this is also part of the avoid set
defenderWin = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
defenderWin = sqrt(defenderWin) - captureRadius;

attackerWin = shapeDifference(attackerWin, defenderWin);

function data = LevelsetFun(g, targetRadius)
    xa = g.xs{1};
    ya = g.xs{2};
    xd = g.xs{3};
    yd = g.xs{4};
    
    % case1: attacker in target area
    Target_a = xa.^2 + ya.^2 - targetRadius^2;
    
    % case2: Not caught by defender
%     Catch = -((xd-xa).^2 + (yd-ya).^2 - captureRadius^2);
    
    % case3: defender not in target area
%     Target_d = -(xd.^2 + yd.^2 - targetRadius^2);
    
    % case4: defender in obstacle
%     Obs_d = ((xd-obsCenter(1)).^2 + (yd-obsCenter(2)).^2 - obsRadius^2);

    % Terminal set: case1 && case2 && case3
%     data = min(max(max(Target_a, Catch), Target_d), Obs_d);
%     data = (max(Target_a, Catch));
    data = Target_a;
    
end
