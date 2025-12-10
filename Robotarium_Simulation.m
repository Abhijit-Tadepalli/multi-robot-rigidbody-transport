function robotarium_maze_carry

clc; close all;

%% -------------------- PARAMETERS --------------------

N = 4;                         
robotRadiusSafety = 0.08;       
objectSize  = 0.20;             

% World limits (for planning + drawing)
worldX = [-1.5, 1.5];
worldY = [-1.0, 1.0];

gridRes = 0.03;                 

% Object start and goal (center positions)
objectStart = [-0.8; -0.6];
objectGoal  = [ 1.0;  0.6];

% Robots start in a line at the bottom-left
x0     = linspace(-1.2, -0.4, N);
y0     = -0.9 * ones(1,N);
theta0 = zeros(1,N);
init_conditions = [x0; y0; theta0];


% distance from object center to robot center =
%   half object + "robot radius" + small gap
gapToObject = 0.02;
contactDist = objectSize/2 + robotRadiusSafety + gapToObject;


% [top, bottom, right, left]
formationOffsets = [...
     0,           0,            contactDist, -contactDist;  % dx
     contactDist, -contactDist, 0,           0            ];% dy


cycle        = 0.05; 
maxIter      = 4000;  
reachTol     = 0.05;  
formationTol = 0.08;  
stepSizeObj  = 0.015; 

dockTol      = 0.08;  

%% -------------------- ROBOTARIUM BACKEND --------------------

r = Robotarium(...
    'NumberOfRobots', N, ...
    'ShowFigure', true, ...
    'InitialConditions', init_conditions);

% Robotarium utilities
si_pos_controller = create_si_position_controller();
si_to_uni_dyn     = create_si_to_uni_dynamics();




poses = r.get_poses();
r.set_velocities(1:N, zeros(2,N));
r.step();

ax = gca;
hold(ax,'on');
axis(ax,'equal');
xlim(ax, worldX);
ylim(ax, worldY);
title(ax,'Robotarium Simulation of Robots');
xlabel(ax,'X'); ylabel(ax,'Y');

%% -------------------- NO OBSTACLES --------------------
obstacles = [];   % free space
plot(objectGoal(1), objectGoal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

%% -------------------- DRAW OBJECT --------------------

objPatch = patch(...
    objectStart(1) + objectSize/2*[-1 -1  1  1], ...
    objectStart(2) + objectSize/2*[-1  1  1 -1], ...
    [0 0 0]);

%% -------------------- PLAN PATH WITH BFS --------------------

waypoints = planPathBFS(objectStart, objectGoal, obstacles, worldX, worldY, gridRes);


plot(ax, waypoints(1,:), waypoints(2,:), '--', 'Color', [0.9 0.9 0.9]);

%% -------------------- PHASE 1: DOCKING --------------------


maxDockSteps = 600;
for step = 1:maxDockSteps
    poses    = r.get_poses();
    robotPos = poses(1:2,:);

    desiredDockPos = objectStart + formationOffsets;

    dxi = si_pos_controller(robotPos, desiredDockPos);

  

    dxu = si_to_uni_dyn(dxi, poses);

    r.set_velocities(1:N, dxu);
    r.step();

    % Object stays at start during docking
    set(objPatch, 'XData', objectStart(1) + objectSize/2*[-1 -1 1 1], ...
                  'YData', objectStart(2) + objectSize/2*[-1  1 1 -1]);

    % Check docking error
    errs = sqrt(sum((robotPos - desiredDockPos).^2, 1));
    if max(errs) < dockTol
        break;
    end

    pause(cycle);
end

%% -------------------- PHASE 2: CARRY ALONG PATH --------------------

objectPos  = objectStart;
currentSeg = 1;
numSegs    = size(waypoints,2) - 1;

for iter = 1:maxIter
    poses    = r.get_poses();
    robotPos = poses(1:2,:);

    % Stop if object at goal
    if norm(objectPos - objectGoal) < reachTol
        r.set_velocities(1:N, zeros(2,N));
        r.step();
        break;
    end

    % --- formation error for current object position ---
    desiredNow = objectPos + formationOffsets;
    errs       = sqrt(sum((robotPos - desiredNow).^2, 1)); 
    maxErr     = max(errs);

    % --- choose segment target ---
    if currentSeg > numSegs
        pTarget = objectGoal;
    else
        pB = waypoints(:,currentSeg+1);
        if norm(objectPos - pB) < 0.05
            currentSeg = currentSeg + 1;
        end
        if currentSeg > numSegs
            pTarget = objectGoal;
        else
            pTarget = waypoints(:,currentSeg+1);
        end
    end

    % --- move object ONLY if robots are in formation ---
    if maxErr < formationTol
        dir  = pTarget - objectPos;
        dist = norm(dir);
        if dist > 1e-6
            dir       = dir / dist;
            objectPos = objectPos + stepSizeObj * dir;
        end
    end

    % update object drawing
    set(objPatch, 'XData', objectPos(1) + objectSize/2*[-1 -1 1 1], ...
                  'YData', objectPos(2) + objectSize/2*[-1  1 1 -1]);

    % desired formation positions around new objectPos
    desiredPos = objectPos + formationOffsets;

    dxi = si_pos_controller(robotPos, desiredPos);

    

    dxu = si_to_uni_dyn(dxi, poses);

    r.set_velocities(1:N, dxu);
    r.step();

    pause(cycle);
end

r.debug();  

end

%% ==================== HELPERS ====================

function waypoints = planPathBFS(startPos, goalPos, obstacles, worldX, worldY, gridRes)


xMin = worldX(1); xMax = worldX(2);
yMin = worldY(1); yMax = worldY(2);

nx = round((xMax - xMin)/gridRes) + 1;
ny = round((yMax - yMin)/gridRes) + 1;

occ = false(ny, nx);

% Obstacles loop does nothing if obstacles is empty
for k = 1:size(obstacles,1)
    xL = obstacles(k,1);
    yB = obstacles(k,2);
    w  = obstacles(k,3);
    h  = obstacles(k,4);

    xR = xL + w;
    yT = yB + h;

    ix1 = max(1, floor((xL - xMin)/gridRes) + 1);
    ix2 = min(nx, ceil((xR - xMin)/gridRes) + 1);
    iy1 = max(1, floor((yB - yMin)/gridRes) + 1);
    iy2 = min(ny, ceil((yT - yMin)/gridRes) + 1);

    occ(iy1:iy2, ix1:ix2) = true;
end

[start_ix, start_iy] = worldToGrid(startPos, xMin, yMin, gridRes);
[goal_ix,  goal_iy ] = worldToGrid(goalPos,  xMin, yMin, gridRes);

visited = false(ny, nx);
parent  = zeros(ny, nx, 2, 'int32');

queue = zeros(nx*ny,2,'int32');
head = 1; tail = 1;

queue(tail,:) = int32([start_iy, start_ix]); tail = tail + 1;
visited(start_iy, start_ix) = true;

dirs = int32([-1 0; 1 0; 0 -1; 0 1]);

found = false;
while head < tail
    cy = queue(head,1);
    cx = queue(head,2);
    head = head + 1;

    if cy == goal_iy && cx == goal_ix
        found = true;
        break;
    end

    for d = 1:4
        nyy = cy + dirs(d,1);
        nxx = cx + dirs(d,2);
        if nyy < 1 || nyy > ny || nxx < 1 || nxx > nx
            continue;
        end
        if visited(nyy,nxx) || occ(nyy,nxx)
            continue;
        end
        visited(nyy,nxx) = true;
        parent(nyy,nxx,:) = [cy, cx];
        queue(tail,:) = [nyy, nxx];
        tail = tail + 1;
    end
end

if ~found
    error('No path found.');
end

% Reconstruct path
pathCells = zeros(nx*ny,2,'int32');
count = 0;
cy = goal_iy; cx = goal_ix;
while ~(cy == start_iy && cx == start_ix)
    count = count + 1;
    pathCells(count,:) = [cy, cx];
    p = parent(cy,cx,:);
    cy = p(1); cx = p(2);
end
count = count + 1;
pathCells(count,:) = [start_iy, start_ix];

pathCells = flipud(pathCells(1:count,:));
pathCells = compressPath(pathCells);

% Convert back to world coordinates
waypoints = zeros(2,size(pathCells,1));
for i = 1:size(pathCells,1)
    iy = double(pathCells(i,1));
    ix = double(pathCells(i,2));
    x = xMin + (ix-1)*gridRes;
    y = yMin + (iy-1)*gridRes;
    waypoints(:,i) = [x; y];
end

end

function [ix, iy] = worldToGrid(pos, xMin, yMin, res)
x = pos(1); y = pos(2);
ix = round((x - xMin)/res) + 1;
iy = round((y - yMin)/res) + 1;
end

function path2 = compressPath(path)
% keep only turning points
if size(path,1) <= 2
    path2 = path; return;
end
path2   = path(1,:);
prevDir = path(2,:) - path(1,:);
for i = 2:(size(path,1)-1)
    dir = path(i+1,:) - path(i,:);
    if any(dir ~= prevDir)
        path2(end+1,:) = path(i,:); 
        prevDir = dir;
    end
end
path2(end+1,:) = path(end,:);
end
