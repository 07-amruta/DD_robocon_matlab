function robocon3DGameplaySimulation_WithSimulink()
    clf; close all;
    figure('Name','Robocon 2025 Path Planning and Navigation','Color','w');
    axis equal; hold on; grid on; view(3);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    xlim([-2000 17000]); ylim([-2000 11000]); zlim([0 1500]);

    modelName = 'omni_shooter_model_final';
    
    [courtLength, courtWidth, centerY, halfLineX] = deal(15000, 9000, 4500, 7500);
    drawCourt3D(courtLength, courtWidth, centerY);

    [botRadius, botHeight] = deal(300, 600);
    
    opp1Pos = [13000, 3000, 300]; opp2Pos = [13000, 6000, 300];
    shooterPos = [3000, centerY, 300];
    jumpBotPos = [3000, centerY - 800, 300];
    
    oppBot1 = createCylinder(opp1Pos, botRadius, botHeight, [1 0 0]);
    oppBot2 = createCylinder(opp2Pos, botRadius, botHeight, [1 0 0]);
    shooterBot = createCylinder(shooterPos, botRadius, botHeight, [0 0 1]);
    jumpBot = createCylinder(jumpBotPos, botRadius, botHeight, [0 0 1]);
    defenseNet1 = createDefenseNet(opp1Pos, 400, 800);
    defenseNet2 = createDefenseNet(opp2Pos, 400, 800);
    shooterRail = createLinearRail([0,0,0], 600, 100);
    ballPatch = createBall([-1000, -1000, -1000], 150);

    mapResolution = 50;
    map = binaryOccupancyMap(courtLength/1000, courtWidth/1000, mapResolution);
    stateSpace = stateSpaceSE2;
    stateSpace.StateBounds = [0 courtLength/1000; 0 courtWidth/1000; -pi pi];
    stateValidator = validatorOccupancyMap(stateSpace);
    stateValidator.Map = map;
    stateValidator.ValidationDistance = 0.1;
    planner = plannerRRTStar(stateSpace, stateValidator);
    planner.MaxConnectionDistance = 2.0;
    planner.MaxIterations = 1000;
    planner.GoalReachedFcn = @(planner, q) norm(q(1:2) - planner.GoalState(1:2)) < 0.5;
    currentNavPath = [];
    navWaypointIndex = 1;
    lastNavUpdate = 0;
    navUpdateInterval = 1.0;

    dt = 0.12; maxTime = 40;
    shootingTarget = [14400, centerY];
    arcCenter = [14400, centerY]; arcRadius = 2500;
    [arcStartAngle, arcEndAngle] = deal(pi, deg2rad(75 + rand()*30));
    [linearSpeed, arcAngularSpeed, oppSpeed] = deal(400, deg2rad(10), 420);
    [evadeDistance, oppAggressionDistance] = deal(1200, 4000);
    minOppYGap = 850;
    collisionRadius = botRadius * 2.2;
    [shooterYaw, jumpBotYaw, phase, hasShot, gameEnded] = deal(0, 0, "toHalfCourt", false, false);
    [currentWaypoint, waypointReached, evasionActive] = deal([], true, false);
    [jumpBotPhase, jumpBotWaypoint, jumpBotWaypointReached] = deal("followShooter", [], true);
    [lastPathUpdate, pathUpdateInterval] = deal(0, 0.2);
    jumpBotTargetPos = [11800, 2650, 300];
    jumpBotArcCenter = [];
    jumpBotArcRadius = 0;
    jumpBotCurrentArcAngle = 0;
    jumpBotArcStartAngle = 0;
    jumpBotArcEndAngle = 0;

    for t = 0:dt:maxTime
        if gameEnded, break; end
        
        try
            switch phase
                case "idle"
                    phase_num = 1;
                case "toHalfCourt"
                    phase_num = 2;
                case "navigateToArc"
                    phase_num = 3;
                case "aroundArc"
                    phase_num = 4;
                case "shoot"
                    phase_num = 5;
                otherwise
                    phase_num = 6; 
            end
            
            set_param([modelName '/shooterPos'], 'Value', mat2str(shooterPos));
            set_param([modelName '/jumpBotPos'], 'Value', mat2str(jumpBotPos));
            set_param([modelName '/opp1Pos'], 'Value', mat2str(opp1Pos));
            set_param([modelName '/opp2Pos'], 'Value', mat2str(opp2Pos));
            set_param([modelName '/phase'], 'Value', num2str(phase_num));
            
        catch ME
            disp(['Simulink update error: ' ME.message]);
        end
        
        
        if t - lastNavUpdate > navUpdateInterval
            setOccupancy(map, [0 courtLength/1000; 0 courtWidth/1000], 0);
            opp1Circle = createCircleObstacle(opp1Pos(1:2)/1000, (botRadius*2)/1000, mapResolution);
            opp2Circle = createCircleObstacle(opp2Pos(1:2)/1000, (botRadius*2)/1000, mapResolution);
            
            for i = 1:size(opp1Circle, 1)
                if opp1Circle(i,1) > 0 && opp1Circle(i,1) <= courtLength/1000 && ...
                   opp1Circle(i,2) > 0 && opp1Circle(i,2) <= courtWidth/1000
                    setOccupancy(map, opp1Circle(i,:), 1);
                end
            end
            
            for i = 1:size(opp2Circle, 1)
                if opp2Circle(i,1) > 0 && opp2Circle(i,1) <= courtLength/1000 && ...
                   opp2Circle(i,2) > 0 && opp2Circle(i,2) <= courtWidth/1000
                    setOccupancy(map, opp2Circle(i,:), 1);
                end
            end
            lastNavUpdate = t;
        end

        distToShooter1 = norm(shooterPos(1:2) - opp1Pos(1:2));
        distToJumpBot2 = norm(jumpBotPos(1:2) - opp2Pos(1:2));
        
        opp1Target = calculateDynamicBlockingPosition(shooterPos, shootingTarget, courtLength, courtWidth, botRadius);
        
        jumpBotDirection = jumpBotTargetPos(1:2) - jumpBotPos(1:2);
        if norm(jumpBotDirection) > 0
            jumpBotDirection = jumpBotDirection / norm(jumpBotDirection);
        else
            jumpBotDirection = [1, 0];
        end
        
        diagonalAngle = atan2(jumpBotDirection(2), jumpBotDirection(1)) + pi/4;
        diagonalDirection = [cos(diagonalAngle), sin(diagonalAngle)];
        blockingDistance = 1000;
        diagonalBlockingPos = jumpBotPos(1:2) + diagonalDirection * blockingDistance;
        
        if diagonalBlockingPos(1) < shooterPos(1) + 800
            diagonalBlockingPos(1) = shooterPos(1) + 800;
        end
        
        distanceToShooter = norm(diagonalBlockingPos - shooterPos(1:2));
        if distanceToShooter < 1200
            awayFromShooter = (diagonalBlockingPos - shooterPos(1:2)) / distanceToShooter;
            diagonalBlockingPos = shooterPos(1:2) + awayFromShooter * 1200;
        end
        
        diagonalBlockingPos(1) = max(min(diagonalBlockingPos(1), courtLength - botRadius - 200), botRadius + 200);
        diagonalBlockingPos(2) = max(min(diagonalBlockingPos(2), courtWidth - botRadius - 200), botRadius + 200);
        opp2Target = [diagonalBlockingPos, 300];
        
        newOpp1Pos = moveOpponentDynamic(opp1Pos, opp1Target, oppSpeed, dt, courtLength, courtWidth, botRadius, shooterPos);
        newOpp2Pos = moveOpponentDynamic(opp2Pos, opp2Target, oppSpeed, dt, courtLength, courtWidth, botRadius, jumpBotPos);
        [opp1Pos, opp2Pos] = resolveCollisions(newOpp1Pos, newOpp2Pos, shooterPos, collisionRadius, opp1Pos, opp2Pos);
        [opp1Yaw, opp2Yaw] = deal(atan2(shooterPos(2) - opp1Pos(2), shooterPos(1) - opp1Pos(1)), ...
                                  atan2(jumpBotPos(2) - opp2Pos(2), jumpBotPos(1) - opp2Pos(1)));

        switch jumpBotPhase
            case "followShooter"
                distanceToShooter = norm(shooterPos(1:2) - jumpBotPos(1:2));
                if distanceToShooter > 1000
                    direction = (shooterPos(1:2) - jumpBotPos(1:2)) / distanceToShooter;
                    jumpBotPos(1:2) = jumpBotPos(1:2) + direction * linearSpeed * dt;
                elseif distanceToShooter < 900
                    direction = (jumpBotPos(1:2) - shooterPos(1:2)) / distanceToShooter;
                    jumpBotPos(1:2) = jumpBotPos(1:2) + direction * linearSpeed * dt;
                end
                
                if strcmp(phase, "aroundArc")
                    jumpBotPhase = "moveToTarget";
                    startPos = jumpBotPos(1:2);
                    endPos = jumpBotTargetPos(1:2);
                    midPoint = (startPos + endPos) / 2;
                    distance = norm(endPos - startPos);
                    arcHeight = distance * 0.3;
                    direction = (endPos - startPos) / distance;
                    perpendicular = [-direction(2), direction(1)];
                    jumpBotArcCenter = midPoint + perpendicular * arcHeight;
                    jumpBotArcRadius = norm(startPos - jumpBotArcCenter);
                    jumpBotArcStartAngle = atan2(startPos(2) - jumpBotArcCenter(2), startPos(1) - jumpBotArcCenter(1));
                    jumpBotArcEndAngle = atan2(endPos(2) - jumpBotArcCenter(2), endPos(1) - jumpBotArcCenter(1));
                    angleDiff = jumpBotArcEndAngle - jumpBotArcStartAngle;
                    if angleDiff > pi
                        jumpBotArcEndAngle = jumpBotArcEndAngle - 2*pi;
                    elseif angleDiff < -pi
                        jumpBotArcEndAngle = jumpBotArcEndAngle + 2*pi;
                    end
                    jumpBotCurrentArcAngle = jumpBotArcStartAngle;
                    jumpBotWaypointReached = false;
                end
                
            case "moveToTarget"
                if ~jumpBotWaypointReached
                    angleStep = arcAngularSpeed * dt;
                    if jumpBotArcEndAngle > jumpBotArcStartAngle
                        jumpBotCurrentArcAngle = min(jumpBotCurrentArcAngle + angleStep, jumpBotArcEndAngle);
                    else
                        jumpBotCurrentArcAngle = max(jumpBotCurrentArcAngle - angleStep, jumpBotArcEndAngle);
                    end
                    jumpBotPos(1:2) = jumpBotArcCenter + jumpBotArcRadius * [cos(jumpBotCurrentArcAngle), sin(jumpBotCurrentArcAngle)];
                    if abs(jumpBotCurrentArcAngle - jumpBotArcEndAngle) < 0.01
                        jumpBotPos = jumpBotTargetPos;
                        jumpBotWaypointReached = true;
                        jumpBotPhase = "blocking";
                    end
                end
                
            case "blocking"
                jumpBotPos = jumpBotTargetPos;
        end
        
        if strcmp(jumpBotPhase, "followShooter")
            jumpBotYaw = atan2(shooterPos(2) - jumpBotPos(2), shooterPos(1) - jumpBotPos(1));
        else
            jumpBotYaw = atan2(opp2Pos(2) - jumpBotPos(2), opp2Pos(1) - jumpBotPos(1));
        end

        % Enhanced Shooter AI with Navigation Toolbox
        ballPos = [-1000, -1000, -1000];
        
        switch phase
            case "toHalfCourt"
                if isempty(currentNavPath) || navWaypointIndex > length(currentNavPath.States)
                    try
                        startState = [shooterPos(1)/1000, shooterPos(2)/1000, shooterYaw];
                        goalState = [(halfLineX - 1000)/1000, centerY/1000, 0];
                        currentNavPath = plan(planner, startState, goalState);
                        navWaypointIndex = 1;
                    catch
                        halfCourtTarget = [halfLineX - 1000, centerY, 300];
                        if t - lastPathUpdate > pathUpdateInterval || waypointReached
                            [currentWaypoint, evasionActive] = planSmartPath(shooterPos, halfCourtTarget, [opp1Pos; opp2Pos], evadeDistance, courtWidth, botRadius);
                            [lastPathUpdate, waypointReached] = deal(t, false);
                        end
                        [shooterPos, waypointReached] = moveToWaypoint(shooterPos, currentWaypoint, linearSpeed, dt, evasionActive);
                    end
                end
                
                if ~isempty(currentNavPath) && navWaypointIndex <= length(currentNavPath.States)
                    targetWaypoint = [currentNavPath.States(navWaypointIndex, 1:2) * 1000, 300];
                    [shooterPos, reached] = moveToWaypoint(shooterPos, targetWaypoint, linearSpeed, dt, false);
                    if reached
                        navWaypointIndex = navWaypointIndex + 1;
                    end
                end
                
                if shooterPos(1) >= halfLineX - 1200
                    phase = "navigateToArc";
                    arcEntryPoint = [arcCenter(1) + arcRadius * cos(arcStartAngle), arcCenter(2) + arcRadius * sin(arcStartAngle), 300];
                    currentNavPath = [];
                    navWaypointIndex = 1;
                end
                
            case "navigateToArc"
                if isempty(currentNavPath) || navWaypointIndex > length(currentNavPath.States)
                    try
                        startState = [shooterPos(1)/1000, shooterPos(2)/1000, shooterYaw];
                        goalState = [arcEntryPoint(1)/1000, arcEntryPoint(2)/1000, 0];
                        currentNavPath = plan(planner, startState, goalState);
                        navWaypointIndex = 1;
                    catch
                        if t - lastPathUpdate > pathUpdateInterval || waypointReached
                            [currentWaypoint, evasionActive] = planSmartPath(shooterPos, arcEntryPoint, [opp1Pos; opp2Pos], evadeDistance, courtWidth, botRadius);
                            [lastPathUpdate, waypointReached] = deal(t, false);
                        end
                        [shooterPos, waypointReached] = moveToWaypoint(shooterPos, currentWaypoint, linearSpeed, dt, evasionActive);
                    end
                end
                
                if ~isempty(currentNavPath) && navWaypointIndex <= length(currentNavPath.States)
                    targetWaypoint = [currentNavPath.States(navWaypointIndex, 1:2) * 1000, 300];
                    [shooterPos, reached] = moveToWaypoint(shooterPos, targetWaypoint, linearSpeed, dt, false);
                    if reached
                        navWaypointIndex = navWaypointIndex + 1;
                    end
                end
                
                if norm(shooterPos(1:2) - arcEntryPoint(1:2)) < 200
                    [shooterPos(1:2), phase, currentArcAngle] = deal(arcEntryPoint(1:2), "aroundArc", arcStartAngle);
                end
                
            case "aroundArc"
                currentArcAngle = max(currentArcAngle - arcAngularSpeed * dt, arcEndAngle);
                shooterPos(1:2) = arcCenter + arcRadius * [cos(currentArcAngle), sin(currentArcAngle)];
                ballPos = shooterPos + [botRadius + 150, 0, 300];
                if currentArcAngle == arcEndAngle && ~hasShot
                    performShotAnimation(ballPatch, shooterPos, shootingTarget, botRadius);
                    hasShot = true;
                    gameEnded = true;
                end
        end
        
        if ~isempty(currentNavPath) && navWaypointIndex <= length(currentNavPath.States)
            targetWaypoint = [currentNavPath.States(navWaypointIndex, 1:2) * 1000, 300];
            if norm(targetWaypoint(1:2) - shooterPos(1:2)) > 50
                shooterYaw = atan2(targetWaypoint(2) - shooterPos(2), targetWaypoint(1) - shooterPos(1));
            end
        elseif ~isempty(currentWaypoint) && norm(currentWaypoint(1:2) - shooterPos(1:2)) > 50
            shooterYaw = atan2(currentWaypoint(2) - shooterPos(2), currentWaypoint(1) - shooterPos(1));
        else
            shooterYaw = atan2(shootingTarget(2) - shooterPos(2), shootingTarget(1) - shooterPos(1));
        end

        updateCylinder(shooterBot, shooterPos, shooterYaw, botRadius, botHeight);
        updateCylinder(jumpBot, jumpBotPos, jumpBotYaw, botRadius, botHeight);
        updateLinearRail(shooterRail, shooterPos, shooterYaw);
        updateCylinder(oppBot1, opp1Pos, opp1Yaw, botRadius, botHeight);
        updateCylinder(oppBot2, opp2Pos, opp2Yaw, botRadius, botHeight);
        updateDefenseNet(defenseNet1, opp1Pos); 
        updateDefenseNet(defenseNet2, opp2Pos);
        updateBall(ballPatch, ballPos);
        drawnow;
    end
    
    if hasShot
        title('Game Complete - Successful Shot with Navigation Toolbox!', 'FontSize', 14, 'Color', 'green');
    end
end

function circlePoints = createCircleObstacle(center, radius, resolution)
    theta = linspace(0, 2*pi, 20);
    x = center(1) + radius * cos(theta);
    y = center(2) + radius * sin(theta);
    circlePoints = [x', y'];
end

function target = calculateDynamicBlockingPosition(botPos, shootingTarget, courtLength, courtWidth, botRadius)
    botToTarget = shootingTarget - botPos(1:2);
    targetDistance = norm(botToTarget);
    
    blockingDistance = min(1200, max(600, targetDistance * 0.4));
    blockingDirection = botToTarget / targetDistance;
    
    blockingPos = botPos(1:2) + blockingDirection * blockingDistance;
    
    lateralOffset = 300 * sin(atan2(botToTarget(2), botToTarget(1)));
    blockingPos = blockingPos + [-lateralOffset * 0.3, lateralOffset];
    
    blockingPos(1) = max(min(blockingPos(1), courtLength - botRadius - 200), botRadius + 200);
    blockingPos(2) = max(min(blockingPos(2), courtWidth - botRadius - 200), botRadius + 200);
    
    target = [blockingPos, 300];
end

function [newOpp1Pos, newOpp2Pos] = resolveCollisions(opp1Pos, opp2Pos, shooterPos, collisionRadius, oldOpp1Pos, oldOpp2Pos)
    dist1ToShooter = norm(opp1Pos(1:2) - shooterPos(1:2));
    dist2ToShooter = norm(opp2Pos(1:2) - shooterPos(1:2));
    
    if dist1ToShooter < collisionRadius
        direction = (opp1Pos(1:2) - shooterPos(1:2)) / dist1ToShooter;
        opp1Pos(1:2) = shooterPos(1:2) + direction * collisionRadius;
    end
    
    if dist2ToShooter < collisionRadius
        direction = (opp2Pos(1:2) - shooterPos(1:2)) / dist2ToShooter;
        opp2Pos(1:2) = shooterPos(1:2) + direction * collisionRadius;
    end
    
    distBetweenOpps = norm(opp1Pos(1:2) - opp2Pos(1:2));
    if distBetweenOpps < collisionRadius
        midPoint = (opp1Pos(1:2) + opp2Pos(1:2)) / 2;
        direction1 = (opp1Pos(1:2) - midPoint);
        direction2 = (opp2Pos(1:2) - midPoint);
        
        if norm(direction1) > 0 && norm(direction2) > 0
            direction1 = direction1 / norm(direction1);
            direction2 = direction2 / norm(direction2);
            opp1Pos(1:2) = midPoint + direction1 * collisionRadius/2;
            opp2Pos(1:2) = midPoint + direction2 * collisionRadius/2;
        else
            opp1Pos = oldOpp1Pos;
            opp2Pos = oldOpp2Pos;
        end
    end
    
    newOpp1Pos = opp1Pos;
    newOpp2Pos = opp2Pos;
end

function newPos = moveOpponentDynamic(currentPos, target, speed, dt, courtLength, courtWidth, botRadius, targetBot)
    direction = target - currentPos;
    distance = norm(direction(1:2));
    if distance <= 10
        newPos = currentPos;
        return;
    end
    
    direction = direction / distance;
    
    targetBotDistance = norm(targetBot(1:2) - currentPos(1:2));
    if targetBotDistance < 800
        moveSpeed = speed * 1.6; 
    elseif targetBotDistance < 1500
        moveSpeed = speed * 1.3; 
    else
        moveSpeed = speed; 
    end
    
    if distance > 300
        actualSpeed = moveSpeed;
    else
        actualSpeed = moveSpeed * (distance / 300);
    end
    
    newPos = currentPos + direction * actualSpeed * dt;
    
    newPos(1) = max(min(newPos(1), courtLength - botRadius), botRadius);
    newPos(2) = max(min(newPos(2), courtWidth - botRadius), botRadius);
end

function [waypoint, evasionActive] = planSmartPath(currentPos, target, opponents, evadeDistance, courtWidth, botRadius)
    evasionActive = false;
    waypoint = target;
    
    directPath = target(1:2) - currentPos(1:2);
    pathLength = norm(directPath);
    if pathLength == 0, return; end
    
    pathDirection = directPath / pathLength;
    
    for i = 1:size(opponents, 1)
        oppPos = opponents(i, 1:2);
        toOpp = oppPos - currentPos(1:2);
        
        projectionLength = dot(toOpp, directPath) / pathLength;
        if projectionLength > 0 && projectionLength < pathLength
            projectionPoint = currentPos(1:2) + (projectionLength / pathLength) * directPath;
            distToPath = norm(oppPos - projectionPoint);
            
            if distToPath < evadeDistance
                evasionActive = true;
                perpendicular = [-pathDirection(2), pathDirection(1)];
                avoidanceDistance = evadeDistance * 1.2;
                
                option1 = oppPos + perpendicular * avoidanceDistance;
                option2 = oppPos - perpendicular * avoidanceDistance;
                
                valid1 = option1(2) >= botRadius + 200 && option1(2) <= courtWidth - botRadius - 200;
                valid2 = option2(2) >= botRadius + 200 && option2(2) <= courtWidth - botRadius - 200;
                
                if valid1 && valid2
                    if norm(option1 - target(1:2)) < norm(option2 - target(1:2))
                        waypoint = [option1, currentPos(3)];
                    else
                        waypoint = [option2, currentPos(3)];
                    end
                elseif valid1
                    waypoint = [option1, currentPos(3)];
                elseif valid2
                    waypoint = [option2, currentPos(3)];
                end
                return;
            end
        end
        
        if norm(currentPos(1:2) - oppPos) < evadeDistance * 0.8
            evasionActive = true;
            avoidVec = (currentPos(1:2) - oppPos) / norm(currentPos(1:2) - oppPos);
            perpVec = [-avoidVec(2), avoidVec(1)];
            
            toTarget = target(1:2) - currentPos(1:2);
            if dot(perpVec, toTarget) > 0
                avoidancePoint = currentPos(1:2) + perpVec * evadeDistance;
            else
                avoidancePoint = currentPos(1:2) - perpVec * evadeDistance;
            end
            avoidancePoint(2) = max(min(avoidancePoint(2), courtWidth - botRadius - 200), botRadius + 200);
            waypoint = [avoidancePoint, currentPos(3)];
            break;
        end
    end
end

function [newPos, reached] = moveToWaypoint(currentPos, waypoint, speed, dt, evasionActive)
    if isempty(waypoint)
        [newPos, reached] = deal(currentPos, true);
        return;
    end
    
    direction = waypoint - currentPos;
    distance = norm(direction(1:2));
    
    if distance < speed * dt
        [newPos, reached] = deal(waypoint, true);
    else
        if evasionActive
            actualSpeed = speed * 1.3;
        else
            actualSpeed = speed;
        end
        newPos = currentPos + (direction / distance) * actualSpeed * dt;
        reached = false;
    end
end

function performShotAnimation(ballPatch, shooterPos, target, botRadius)
    ballStart = shooterPos + [botRadius + 150, 0, 300];
    ballEnd = [target(1), target(2), 3050];
    for i = 1:30
        tNorm = i / 30;
        ballPos = (1 - tNorm) * ballStart + tNorm * ballEnd;
        ballPos(3) = ballPos(3) + 4000 * tNorm * (1 - tNorm);
        updateBall(ballPatch, ballPos);
        drawnow; pause(0.01);
    end
end

function drawCourt3D(courtLength, courtWidth, centerY)
    clf; hold on; axis equal; grid off;
    
    fill3([0 0 courtLength courtLength], [0 courtWidth courtWidth 0], [0 0 0 0], [0.85 0.85 0.85], 'EdgeColor', 'none');
    borderW = 150; borderColor = [90 45 0]/255;
    
    borders_x = [-borderW 0 0 -borderW; courtLength courtLength+borderW courtLength+borderW courtLength; 0 courtLength courtLength 0; 0 courtLength courtLength 0];
    borders_y = [0 0 courtWidth courtWidth; 0 0 courtWidth courtWidth; -borderW -borderW 0 0; courtWidth courtWidth courtWidth+borderW courtWidth+borderW];
    borders_z = zeros(4,4);
    
    for i = 1:4
        fill3(borders_x(i,:), borders_y(i,:), borders_z(i,:), borderColor, 'EdgeColor', 'none');
    end
    
    keyW = 4000; keyH = 2500; y1 = centerY - keyW/2; y2 = centerY + keyW/2;
    fill3([0 keyH keyH 0], [y1 y1 y2 y2], [2 2 2 2], [0.6 0.6 0.6], 'EdgeColor', 'none');
    fill3([courtLength-keyH courtLength courtLength courtLength-keyH], [y1 y1 y2 y2], [2 2 2 2], [0.6 0.6 0.6], 'EdgeColor', 'none');
    
    theta = linspace(-pi/2, pi/2, 100); 
    arcRadius = 3100;
    plot3(600 + arcRadius*cos(theta), centerY + arcRadius*sin(theta), ones(1,100)*2, 'w', 'LineWidth', 3);
    plot3(courtLength - 600 + arcRadius*cos(theta + pi), centerY + arcRadius*sin(theta + pi), ones(1,100)*2, 'w', 'LineWidth', 3);
    
    fill3([-1500 0 0 -1500], [0 0 courtWidth courtWidth], [0 0 0 0], [102 153 255]/255, 'EdgeColor', 'none');
    fill3([courtLength courtLength+1500 courtLength+1500 courtLength], [0 0 courtWidth courtWidth], [0 0 0 0], [255 112 85]/255, 'EdgeColor', 'none');
    
    plot3([courtLength/2 courtLength/2], [0 courtWidth], [0 0], 'w', 'LineWidth', 1.5);
    
    drawHoopAndBackboard([600 centerY 3050]);
    drawHoopAndBackboard([courtLength-600 centerY 3050]);
    
    xlim([-2000 courtLength + 2000]); ylim([-2000 courtWidth + 2000]); zlim([0 4000]);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title('Robocon 2025 Path Planning and Navigation');
    view(310, 25); axis off;
    camlight('headlight'); camlight(-45, 30); lighting gouraud;
end

function drawHoopAndBackboard(center)
    draw3DTorus(center, 450, 40, [1 0.5 0]);
    
    [backboardWidth, backboardHeight, backboardDepth] = deal(1800, 1050, 100);
    if center(1) < 10000
        backboardCorner = [center(1) - backboardDepth, center(2) - backboardWidth/2, center(3) - backboardHeight/2];
        poleX = center(1) - 250;
    else
        backboardCorner = [center(1) + backboardDepth, center(2) - backboardWidth/2, center(3) - backboardHeight/2];
        poleX = center(1) + 250;
    end
    
    draw3DBox(backboardCorner, backboardDepth, backboardHeight, backboardWidth, [0.95 0.95 0.95], 0.3);
    drawCylinder([poleX, center(2), 0], 75, center(3) + 200, [0.5 0.5 0.5]);
end

function draw3DBox(corner, width, height, depth, color, alpha)
    X = [0 width width 0 0 width width 0] + corner(1);
    Y = [0 0 depth depth 0 0 depth depth] + corner(2);
    Z = [0 0 0 0 height height height height] + corner(3);
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices', [X' Y' Z'], 'Faces', faces, 'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', alpha);
end

function draw3DTorus(center, radius, thickness, color)
    [nCirc, nTube] = deal(30, 15); 
    [theta, phi] = deal(linspace(0, 2*pi, nCirc), linspace(0, 2*pi, nTube));
    [Theta, Phi] = meshgrid(theta, phi);
    X = (radius + thickness * cos(Phi)) .* cos(Theta) + center(1);
    Y = (radius + thickness * cos(Phi)) .* sin(Theta) + center(2);
    Z = thickness * sin(Phi) + center(3);
    surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none');
end

function drawCylinder(pos, radius, height, color)
    [X, Y, Z] = cylinder(radius, 20); 
    surf(X + pos(1), Y + pos(2), Z * height + pos(3), 'FaceColor', color, 'EdgeColor', 'none');
end

function h = createCylinder(center, radius, height, color)
    [x, y, z] = cylinder(radius, 20); 
    h = surf(x + center(1), y + center(2), z * height + center(3) - height/2, 'FaceColor', color, 'EdgeColor', 'none');
end

function updateCylinder(h, center, yaw, radius, height)
    [x, y, z] = cylinder(radius, 20);
    pts = [x(:), y(:)]';
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    pts_rot = R * pts;
    set(h, 'XData', reshape(pts_rot(1,:), size(x)) + center(1), ...
           'YData', reshape(pts_rot(2,:), size(y)) + center(2), ...
           'ZData', z * height + center(3) - height/2);
end

function h = createDefenseNet(center, radius, height)
    [x, y, z] = cylinder(radius, 20);
    h = surf(x + center(1), y + center(2), z * height + center(3) - height/2, ...
             'FaceColor', [0 0 0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
end

function updateDefenseNet(h, center)
    currentX = mean(mean(get(h, 'XData')));
    currentY = mean(mean(get(h, 'YData')));
    currentZ = mean(mean(get(h, 'ZData')));
    
    dx = center(1) - currentX;
    dy = center(2) - currentY;
    dz = center(3) - currentZ;
    
    set(h, 'XData', get(h, 'XData') + dx, ...
           'YData', get(h, 'YData') + dy, ...
           'ZData', get(h, 'ZData') + dz);
end

function h = createLinearRail(center, width, thickness)
    x = [-width/2 width/2 width/2 -width/2];
    y = [-thickness/2 -thickness/2 thickness/2 thickness/2];
    h = fill3(center(1) + x, center(2) + y, center(3) * ones(size(x)), [0.7 0.7 0.7]);
end

function updateLinearRail(h, center, yaw)
    width = 600; thickness = 100;
    x = [-width/2 width/2 width/2 -width/2];
    y = [-thickness/2 -thickness/2 thickness/2 thickness/2];
    
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    pts = R * [x; y];
    
    set(h, 'XData', pts(1,:) + center(1), ...
           'YData', pts(2,:) + center(2), ...
           'ZData', center(3) * ones(size(x)));
end

function h = createBall(center, radius)
    [x, y, z] = sphere(15); 
    h = surf(x * radius + center(1), y * radius + center(2), z * radius + center(3), ...
             'FaceColor', [1 0.5 0], 'EdgeColor', 'none', 'FaceAlpha', 1);
end

function updateBall(h, center)
    radius = 150;
    [x, y, z] = sphere(15);
    set(h, 'XData', x * radius + center(1), ...
           'YData', y * radius + center(2), ...
           'ZData', z * radius + center(3));
end
