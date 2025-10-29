function create_omni_shooter_model_final()
    modelName = 'omni_shooter_model_final';

    % ✅ If model already exists, open it and exit
    if exist([modelName '.slx'], 'file')
        if ~bdIsLoaded(modelName)
            open_system(modelName);
        end
        disp("📂 Existing model opened: " + modelName);
        return;
    end

    % ✅ Otherwise, create a new model from scratch
    new_system(modelName);
    open_system(modelName);

    x0 = 30; y0 = 40; dx = 300;

    %% Add Input Ports for real-time data
    add_block('simulink/Sources/In1', [modelName '/shooterPos'], 'Position', [x0-150 y0 x0-100 y0+20]);
    add_block('simulink/Sources/In1', [modelName '/jumpBotPos'], 'Position', [x0-150 y0+30 x0-100 y0+50]);
    add_block('simulink/Sources/In1', [modelName '/opp1Pos'], 'Position', [x0-150 y0+60 x0-100 y0+80]);
    add_block('simulink/Sources/In1', [modelName '/opp2Pos'], 'Position', [x0-150 y0+90 x0-100 y0+110]);
    add_block('simulink/Sources/In1', [modelName '/currentArcAngle'], 'Position', [x0-150 y0+120 x0-100 y0+140]);
    add_block('simulink/Sources/In1', [modelName '/phase'], 'Position', [x0-150 y0+150 x0-100 y0+170]);

    %% Constants
    add_block('simulink/Sources/Constant', [modelName '/halfLineX'], 'Position', [x0-150 y0+180 x0-100 y0+200], 'Value', '7500');
    add_block('simulink/Sources/Constant', [modelName '/arcEntryPoint'], 'Position', [x0-150 y0+210 x0-100 y0+230], 'Value', '[11900 4500]');
    add_block('simulink/Sources/Constant', [modelName '/arcEndAngle'], 'Position', [x0-150 y0+240 x0-100 y0+260], 'Value', '1.31');

    %% 1. SupervisoryLogic Subsystem + Stateflow
    sfX = x0; sfY = y0 + 200;
    supervisoryPath = [modelName '/SupervisoryLogic'];
    add_block('built-in/SubSystem', supervisoryPath, 'Position', [sfX sfY sfX+250 sfY+180]);

    %% 2. Navigation Subsystem
    navX = x0 + dx;
    add_block('built-in/SubSystem', [modelName '/Navigation'], 'Position', [navX y0 navX+200 y0+120]);
    navFuncPath = [modelName '/Navigation/NavigateLogic'];
    add_block('simulink/User-Defined Functions/MATLAB Function', navFuncPath, 'Position', [navX+40 y0+30 navX+180 y0+80]);

    %% 3. ShootingMechanism Subsystem
    shootX = navX + dx;
    add_block('built-in/SubSystem', [modelName '/ShootingMechanism'], 'Position', [shootX y0 shootX+220 y0+130]);
    shootFuncPath = [modelName '/ShootingMechanism/ShootLogic'];
    add_block('simulink/User-Defined Functions/MATLAB Function', shootFuncPath, 'Position', [shootX+40 y0+30 shootX+200 y0+90]);

    %% 4. OmniDriveKinematics Subsystem
    add_block('built-in/SubSystem', [modelName '/OmniDriveKinematics'], 'Position', [x0 y0 x0+150 y0+100]);

    %% Add Output Display Blocks
    add_block('simulink/Sinks/Display', [modelName '/Mode_Display'], 'Position', [sfX+300 sfY sfX+350 sfY+30]);
    add_block('simulink/Sinks/Display', [modelName '/ShotType_Display'], 'Position', [sfX+300 sfY+40 sfX+350 sfY+70]);
    add_block('simulink/Sinks/Display', [modelName '/VelCmd_Display'], 'Position', [navX+250 y0 navX+300 y0+30]);

    %% Create and configure Stateflow chart (same as your original code)
    open_system(supervisoryPath);
    set_param(modelName, 'SimulationCommand', 'update');

    % Wait for Stateflow initialization
    r = sfroot;
    maxWait = 10; t0 = tic;
    while isempty(r.find('-isa', 'Stateflow.Model', 'Name', modelName))
        pause(0.2);
        if toc(t0) > maxWait
            error('❌ Stateflow model did not initialize in time.');
        end
        r = sfroot;
    end

    % Add Supervisory Chart with robocon states
    m = r.find('-isa', 'Stateflow.Model', 'Name', modelName);
    block = m.find('-isa', 'Stateflow.Block', 'Path', supervisoryPath);
    chart = Stateflow.Chart(block);
    chart.Name = 'SupervisoryChart';

    % Create states
    idle = Stateflow.State(chart);
    idle.Name = 'IDLE';
    idle.Position = [50 50 80 50];
    idle.LabelString = 'IDLE' + newline + 'entry: mode = "idle"; shot_type = "";';

    toHalfCourt = Stateflow.State(chart);
    toHalfCourt.Name = 'TO_HALF_COURT';
    toHalfCourt.Position = [180 50 120 50];
    toHalfCourt.LabelString = 'TO_HALF_COURT' + newline + 'entry: mode = "toHalfCourt"; shot_type = "";';

    navigateToArc = Stateflow.State(chart);
    navigateToArc.Name = 'NAVIGATE_TO_ARC';
    navigateToArc.Position = [350 50 130 50];
    navigateToArc.LabelString = 'NAVIGATE_TO_ARC' + newline + 'entry: mode = "navigateToArc"; shot_type = "";';

    aroundArc = Stateflow.State(chart);
    aroundArc.Name = 'AROUND_ARC';
    aroundArc.Position = [530 50 100 50];
    aroundArc.LabelString = 'AROUND_ARC' + newline + 'entry: mode = "aroundArc"; shot_type = "";';

    shoot = Stateflow.State(chart);
    shoot.Name = 'SHOOT';
    shoot.Position = [680 50 80 50];
    shoot.LabelString = 'SHOOT' + newline + 'entry: mode = "shoot"; shot_type = "arc";';

    gameComplete = Stateflow.State(chart);
    gameComplete.Name = 'GAME_COMPLETE';
    gameComplete.Position = [350 150 120 50];
    gameComplete.LabelString = 'GAME_COMPLETE' + newline + 'entry: mode = "complete"; shot_type = "";';

    % Add transitions
    t1 = Stateflow.Transition(chart);
    t1.Source = idle; t1.Destination = toHalfCourt;
    t1.LabelString = 'after(1,sec)';

    t2 = Stateflow.Transition(chart);
    t2.Source = toHalfCourt; t2.Destination = navigateToArc;
    t2.LabelString = '[strcmp(phase, "navigateToArc")]';

    t3 = Stateflow.Transition(chart);
    t3.Source = navigateToArc; t3.Destination = aroundArc;
    t3.LabelString = '[strcmp(phase, "aroundArc")]';

    t4 = Stateflow.Transition(chart);
    t4.Source = aroundArc; t4.Destination = shoot;
    t4.LabelString = '[strcmp(phase, "shoot")]';

    t5 = Stateflow.Transition(chart);
    t5.Source = shoot; t5.Destination = gameComplete;
    t5.LabelString = 'after(3,sec)';

    t6 = Stateflow.Transition(chart);
    t6.Source = gameComplete; t6.Destination = idle;
    t6.LabelString = 'after(5,sec)';

    % Set MATLAB function scripts
    navChart = sfroot.find('-isa','Stateflow.EMChart','Path',navFuncPath);
    if ~isempty(navChart)
        navChart.Script = [
            "function vel_cmd = NavigateLogic(shooterPos, jumpBotPos, opp1Pos, opp2Pos, mode)" newline ...
            "%#codegen" newline ...
            "vel_cmd = zeros(2,1);" newline ...
            "halfLineX = 7500; centerY = 4500; linearSpeed = 400;" newline ...
            "arcCenter = [14400, 4500]; arcRadius = 2500; arcStartAngle = pi;" newline ...
            "switch mode" newline ...
            "    case ""toHalfCourt""" newline ...
            "        targetPos = [halfLineX - 1000; centerY];" newline ...
            "        direction = targetPos - [shooterPos(1); shooterPos(2)];" newline ...
            "        if norm(direction) > 0" newline ...
            "            vel_cmd = (direction / norm(direction)) * linearSpeed;" newline ...
            "        end" newline ...
            "    case ""navigateToArc""" newline ...
            "        arcEntryPoint = [arcCenter(1) + arcRadius * cos(arcStartAngle); arcCenter(2) + arcRadius * sin(arcStartAngle)];" newline ...
            "        direction = arcEntryPoint - [shooterPos(1); shooterPos(2)];" newline ...
            "        if norm(direction) > 0" newline ...
            "            vel_cmd = (direction / norm(direction)) * linearSpeed;" newline ...
            "        end" newline ...
            "    case ""aroundArc""" newline ...
            "        currentAngle = atan2(shooterPos(2) - arcCenter(2), shooterPos(1) - arcCenter(1));" newline ...
            "        vel_cmd = [-sin(currentAngle); cos(currentAngle)] * linearSpeed * 0.5;" newline ...
            "end" newline ...
            "end"
        ];
    end

    shootChart = sfroot.find('-isa','Stateflow.EMChart','Path',shootFuncPath);
    if ~isempty(shootChart)
        shootChart.Script = [
            "function [shoot_vel, pitch] = ShootLogic(shot_type)" newline ...
            "%#codegen" newline ...
            "shoot_vel = 0; pitch = 0;" newline ...
            "switch shot_type" newline ...
            "    case ""arc""" newline ...
            "        shoot_vel = 4000; pitch = 45;" newline ...
            "end" newline ...
            "end"
        ];
    end

    save_system(modelName);
    disp("✅ Simulink model created and configured for gameplay integration");
end
