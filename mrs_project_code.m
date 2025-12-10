function multi_robot_rigidbody_maze_demo

close all; clc;

%% WORLD AND GRID PARAMETERS
worldX  = [-5, 10];    % x-limits
worldY  = [-4,  6];    % y-limits
gridRes = 0.1;         % occupancy grid resolution

%% OBSTACLES (maze-like)
% Rectangles: [x_left, y_bottom, width, height]
obstacles = [ ...
    -1.5, -2.5, 2.0, 1.0;    % bottom left block
     2.0, -2.5, 2.0, 1.0;    % bottom middle block
     5.0, -2.5, 2.0, 1.0;    % bottom right block
     3.0, -1.0, 1.0, 3.5;    % vertical wall 1
     6.0,  0.0, 1.0, 3.5;    % vertical wall 2
     1.0,  2.5, 2.0, 1.0;    % upper block 1
     5.0,  2.5, 2.0, 1.0];   % upper block 2

%% RIGID BODY (OBJECT + 4 ROBOTS AT VERTICES)

numRobots   = 4;
robotRadius = 0.25;     
objectSize  = 1.2;      

% Initial object center
objectStart = [0; 0];

% Define initial vertex positions for a square around objectStart
half = objectSize / 2;
vertInit = [ -half, -half,  half,  half;   % x coordinates
             -half,  half,  half, -half];  % y coordinates
vertInit = vertInit + objectStart;         % shift to objectStart

% All vertices have mass 1 for simplicity
massList = ones(1, numRobots);

% Create rigid body
rb = pkgMechanics.RigidBodyPlanar(massList, vertInit);
rb.translation.velocity = [0; 0];   % start at rest

% Get initial COM from rb
rb.getStates;
com0 = rb.translation.position(:);  
objectStart = com0;                  

%% SIMULATION PARAMETERS (FOR FORCE CONTROL)

cycle      = 0.05;      % time step (s)
vNominal   = 0.7;       % nominal COM speed along path (m/s)
Kp         = 0.8;       % proportional gain on velocity error
Ki         = 1.0;       % integral gain on velocity error
maxForce   = 5.0;       % per-component saturation on force [N]

forceInt   = zeros(2, numRobots);  % integral of velocity error (per vertex)
maxSteps   = 4000;                 % safety cap on iterations
reachTol   = 0.15;                 % distance threshold to switch to next waypoint

% Preallocate logs for analysis / plots
timeLog        = nan(1, maxSteps);
comLog         = nan(2, maxSteps);
goalDistLog    = nan(1, maxSteps);
velErrNormLog  = nan(1, maxSteps);
segIdxLog      = nan(1, maxSteps);

% Extra logs for theoretical properties
segDistLog     = nan(1, maxSteps);                % distance to current segment waypoint
vDesLog        = nan(2, maxSteps);                % desired COM velocity
vertVelLog     = nan(2, numRobots, maxSteps);     % vertex velocities
forceLog       = nan(2, numRobots, maxSteps);     % vertex forces

%% FIGURE SETUP

fig = figure('Name', 'RigidBody maze + PI force control', ...
             'NumberTitle', 'off');
ax = axes('Parent', fig);
axes(ax);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X'); ylabel(ax, 'Y');
title(ax, 'Click goal, then watch rigid body follow planned path');

xlim(ax, worldX);
ylim(ax, worldY);

% Draw obstacles
for k = 1:size(obstacles, 1)
    rectangle('Position', obstacles(k, :), ...
              'FaceColor', [0.8 0.8 0.8], ...
              'EdgeColor', 'k');
end

% Draw initial rigid body polygon + robots as circles
rb.getStates;
vertPos = [rb.verticeList.position];   

objectPatch = patch(vertPos(1,:), vertPos(2,:), [0 0 0]);  

robotPatches = gobjects(numRobots, 1);
robotLabels  = gobjects(numRobots, 1);
colors       = lines(numRobots);

for i = 1:numRobots
    pos = vertPos(:, i);
    robotPatches(i) = rectangle('Position', ...
        [pos(1) - robotRadius, pos(2) - robotRadius, 2*robotRadius, 2*robotRadius], ...
        'Curvature', [1 1], ...
        'FaceColor', colors(i,:), ...
        'EdgeColor', 'none');
    robotLabels(i) = text(pos(1), pos(2), sprintf('R%d', i), ...
                          'HorizontalAlignment', 'center', ...
                          'VerticalAlignment', 'middle', ...
                          'FontWeight', 'bold', ...
                          'Color', 'w');
end

% Plot COM path over time
comTrail  = com0;
comPlot   = plot(ax, comTrail(1), comTrail(2), 'b-', 'LineWidth', 1.5);

drawnow;

%% ASK USER FOR GOAL POSITION
axes(ax);
[gx, gy] = ginput(1);
objectGoal = [gx; gy];
plot(ax, objectGoal(1), objectGoal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

fprintf('Planning path with BFS...\n');

%% PLAN PATH FOR COM (BFS ON GRID WITH INFLATED OBSTACLES)
waypoints = planPath(objectStart', objectGoal', obstacles, worldX, worldY, gridRes);

% Draw the path as a visible dashed line
plot(ax, waypoints(:,1), waypoints(:,2), 'm--', 'LineWidth', 1.2);

fprintf('Following path with RigidBody + PI control...\n');

%% FOLLOW PATH WITH RIGID BODY + PI FORCE CONTROL

currentSegment = 1;
numSegs        = size(waypoints, 1) - 1;

for step = 1:maxSteps

    if ~ishandle(fig), break; end  

    % Read current state
    rb.getStates;
    com = rb.translation.position(:);     

    % Distance to final goal
    distToGoal = norm(com - objectGoal);

    % Logging for analysis
    timeLog(step)       = step * cycle;
    comLog(:, step)     = com;
    goalDistLog(step)   = distToGoal;
    segIdxLog(step)     = currentSegment;

    % Check if we are at the final goal
    if distToGoal < reachTol
        fprintf('Reached goal at t = %.2f s\n', step*cycle);
        break;
    end

    % Advance along waypoints if close to the next segment target
    if currentSegment < numSegs
        segTarget = waypoints(currentSegment + 1, :)';
        distToSegTarget = norm(com - segTarget);
        if distToSegTarget < reachTol
            currentSegment = currentSegment + 1;
        end
    end

    % Get current local segment target
    if currentSegment >= numSegs
        segTarget = objectGoal;
    else
        segTarget = waypoints(currentSegment + 1, :)';
    end

    % Desired COM velocity (towards segTarget)
    dir  = segTarget - com;
    dist = norm(dir);
    if dist > 1e-6
        dir = dir / dist;
    else
        dir = [0; 0];
    end

    vDesCOM = vNominal * dir;                 
    vDesAll = repmat(vDesCOM, 1, numRobots);  

    % Vertex velocities from RigidBody 
    vertVel = [rb.verticeList.velocity];  

    % PI CONTROL ON VELOCITY ERROR (per vertex)
    % Define error as e = v_des - v_actual
    velError = vDesAll - vertVel;        

    % Integrate error over time
    forceInt = forceInt + velError * cycle;

    % PI control: u = Kp*e + Ki*integral(e)
    forceCtrlList = Kp * velError + Ki * forceInt;   

    % Saturate forces per component to model actuator limits
    forceCtrlList = min(max(forceCtrlList, -maxForce), maxForce);

    % Extra logging for proofs
    segDistLog(step)     = norm(com - segTarget);       
    vDesLog(:, step)     = vDesCOM;
    vertVelLog(:,:,step) = vertVel;
    forceLog(:,:,step)   = forceCtrlList;

    % Log velocity error norm (for plots)
    velErrNormLog(step) = norm(velError(:));

    % Update rigid body dynamics
    rb.updateStates(forceCtrlList, cycle);

    % Visual update

    % Update vertices and polygon
    rb.getStates;
    vertPos = [rb.verticeList.position];   

    set(objectPatch, 'XData', vertPos(1,:), 'YData', vertPos(2,:));

    % Update robots at vertices
    for i = 1:numRobots
        pos = vertPos(:, i);
        set(robotPatches(i), 'Position', ...
            [pos(1) - robotRadius, pos(2) - robotRadius, 2*robotRadius, 2*robotRadius]);
        set(robotLabels(i), 'Position', pos');
    end

    % Update COM trail
    comTrail(:, end+1) = com;
    set(comPlot, 'XData', comTrail(1,:), 'YData', comTrail(2,:));

    drawnow;
    pause(cycle);
end

fprintf('Simulation finished.\n');

%% Trim logs to actual length
lastStep = find(~isnan(timeLog), 1, 'last');
if isempty(lastStep)
    return;
end

timeLog       = timeLog(1:lastStep);
comLog        = comLog(:, 1:lastStep);
goalDistLog   = goalDistLog(1:lastStep);
velErrNormLog = velErrNormLog(1:lastStep);
segIdxLog     = segIdxLog(1:lastStep);

segDistLog    = segDistLog(1:lastStep);
vDesLog       = vDesLog(:, 1:lastStep);
vertVelLog    = vertVelLog(:,:,1:lastStep);
forceLog      = forceLog(:,:,1:lastStep);

%% Post-simulation plots for the report

% 1) COM trajectory vs planned path
figure('Name', 'COM trajectory');
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y');
title('Rigid body COM trajectory vs planned path');
plot(waypoints(:,1), waypoints(:,2), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Planned path');
plot(comLog(1,:), comLog(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'COM trajectory');
plot(objectStart(1), objectStart(2), 'go', 'MarkerSize', 8, 'DisplayName', 'Start');
plot(objectGoal(1),  objectGoal(2),  'rx', 'MarkerSize', 8, 'DisplayName', 'Goal');
legend('Location', 'best');

% 2) Distance to final goal vs time
figure('Name', 'Distance to final goal over time');
plot(timeLog, goalDistLog, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Distance to final goal [m]');
title('Goal distance vs time');

% 3) Velocity error norm vs time  (Property 1, aggregate)
figure('Name', 'Velocity error norm over time');
plot(timeLog, velErrNormLog, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('||v_{des} - v||_2 (all vertices)');
title('Velocity tracking error vs time');

% 4) Distance to current segment waypoint vs time  (Property 2)
figure('Name', 'Distance to current waypoint over time');
plot(timeLog, segDistLog, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Distance to current waypoint [m]');
title('Segment-wise progress toward waypoint');

% 5) Vertex vs desired velocity for Robot 1  (Property 1, detailed)
v1x = squeeze(vertVelLog(1,1,:));
v1y = squeeze(vertVelLog(2,1,:));

figure('Name', 'Vertex 1 velocity vs desired COM velocity');
subplot(2,1,1);
plot(timeLog, vDesLog(1,:), 'k--', 'LineWidth', 1.2); hold on;
plot(timeLog, v1x, 'b-', 'LineWidth', 1.2);
grid on;
ylabel('v_x [m/s]');
legend('v_{des,x}', 'v_{1,x}', 'Location', 'best');
title('Vertex 1 velocity tracking');

subplot(2,1,2);
plot(timeLog, vDesLog(2,:), 'k--', 'LineWidth', 1.2); hold on;
plot(timeLog, v1y, 'b-', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('v_y [m/s]');
legend('v_{des,y}', 'v_{1,y}', 'Location', 'best');

% 6) Force magnitudes at each robot (for actuator discussion)
figure('Name', 'Robot force magnitudes over time');
hold on; grid on;
for i = 1:numRobots
    fx = squeeze(forceLog(1,i,:));
    fy = squeeze(forceLog(2,i,:));
    fm = sqrt(fx.^2 + fy.^2);
    plot(timeLog, fm, 'LineWidth', 1.2, 'DisplayName', sprintf('Robot %d', i));
end
xlabel('Time [s]');
ylabel('||u_i|| [N]');
title('Vertex force magnitudes over time');
legend('Location', 'best');

end % main function

%% Helper: Path planning using BFS on an inflated grid
function waypoints = planPath(startPos, goalPos, obstacles, worldX, worldY, gridRes)
% startPos, goalPos: [x y]
xMin = worldX(1); xMax = worldX(2);
yMin = worldY(1); yMax = worldY(2);

nx = round((xMax - xMin) / gridRes) + 1;
ny = round((yMax - yMin) / gridRes) + 1;

% Occupancy grid: true = occupied
occ = false(ny, nx);

% Inflate obstacles by safety margin to account for object + robots
safetyMargin = 0.8;   

for k = 1:size(obstacles, 1)
    rect = obstacles(k, :);  

    % Original rectangle
    x1 = rect(1);
    y1 = rect(2);
    x2 = rect(1) + rect(3);
    y2 = rect(2) + rect(4);

    % Inflate
    x1 = x1 - safetyMargin;
    y1 = y1 - safetyMargin;
    x2 = x2 + safetyMargin;
    y2 = y2 + safetyMargin;

    % Convert to grid indices
    ix1 = max(1, floor((x1 - xMin) / gridRes) + 1);
    ix2 = min(nx, ceil((x2 - xMin) / gridRes) + 1);
    iy1 = max(1, floor((y1 - yMin) / gridRes) + 1);
    iy2 = min(ny, ceil((y2 - yMin) / gridRes) + 1);

    occ(iy1:iy2, ix1:ix2) = true;
end

% Convert start/goal to grid
[startIx, startIy] = worldToGrid(startPos(1), startPos(2), xMin, yMin, gridRes);
[goalIx,  goalIy ] = worldToGrid(goalPos(1),  goalPos(2),  xMin, yMin, gridRes);

if occ(startIy, startIx)
    error('Start is inside an inflated obstacle.');
end
if occ(goalIy, goalIx)
    error('Goal is inside an inflated obstacle.');
end

% BFS
visited = false(ny, nx);
parent  = zeros(ny, nx, 2, 'int32');

queue = zeros(nx*ny, 2, 'int32');
head  = 1;
tail  = 1;

queue(tail, :) = int32([startIy, startIx]); tail = tail + 1;
visited(startIy, startIx) = true;

dirs = int32([ -1  0;   % up
               1   0;   % down
               0  -1;   % left
               0   1]); % right

found = false;
while head < tail
    cy = queue(head, 1);
    cx = queue(head, 2);
    head = head + 1;

    if cy == goalIy && cx == goalIx
        found = true;
        break;
    end

    for d = 1:4
        nyy = cy + dirs(d,1);
        nxx = cx + dirs(d,2);

        if nyy < 1 || nyy > ny || nxx < 1 || nxx > nx
            continue;
        end
        if visited(nyy, nxx) || occ(nyy, nxx)
            continue;
        end

        visited(nyy, nxx) = true;
        parent(nyy, nxx, :) = [cy, cx];
        queue(tail, :) = [nyy, nxx];
        tail = tail + 1;
    end
end

if ~found
    error('No path found (try lowering safetyMargin or changing obstacles).');
end

% Reconstruct path (from goal back to start)
pathCells = zeros(nx*ny, 2, 'int32');
count = 0;
cy = goalIy;
cx = goalIx;
while ~(cy == startIy && cx == startIx)
    count = count + 1;
    pathCells(count, :) = [cy, cx];
    p = parent(cy, cx, :);
    cy = p(1);
    cx = p(2);
end
count = count + 1;
pathCells(count, :) = [startIy, startIx];

pathCells = flipud(pathCells(1:count, :));

% Compress to turning points
pathCells = compressPath(pathCells);

% Convert cells to world coordinates
nPath = size(pathCells, 1);
waypoints = zeros(nPath, 2);
for i = 1:nPath
    iy = double(pathCells(i,1));
    ix = double(pathCells(i,2));
    x  = xMin + (ix - 1) * gridRes;
    y  = yMin + (iy - 1) * gridRes;
    waypoints(i, :) = [x, y];
end

end

%% Helper: world -> grid index
function [ix, iy] = worldToGrid(x, y, xMin, yMin, res)
ix = round((x - xMin) / res) + 1;
iy = round((y - yMin) / res) + 1;
end

%% Helper: keep only turning points in BFS path
function path2 = compressPath(path)
% path is N x 2 of [iy, ix]
if size(path,1) <= 2
    path2 = path;
    return;
end

path2   = path(1, :);
prevDir = path(2, :) - path(1, :);

for i = 2:(size(path,1)-1)
    dir = path(i+1, :) - path(i, :);
    if any(dir ~= prevDir)
        path2(end+1, :) = path(i, :); 
        prevDir = dir;
    end
end

path2(end+1, :) = path(end, :);
end
