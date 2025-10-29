function build_simulink_model()
    % Create or open model
    modelName = 'ShootingControlSystem_working';
    
    % Check if the model exists on disk first
    if ~exist([modelName '.slx'], 'file') && ~exist([modelName '.mdl'], 'file')
        % Only create a new system if the model doesn't exist
        open_system(new_system(modelName));
        needsSetup = true;
    else
        % Just open the existing model if it exists
        try
            open_system(modelName);
            needsSetup = false;
            disp('✅ Existing model opened. Keeping your changes.');
            return;  % Exit the function - don't modify the existing model
        catch
            % If opening fails, create a new one
            open_system(new_system(modelName));
            needsSetup = true;
        end
    end

    if needsSetup
        %% 1. Define Bus Object (only once)
        if ~evalin('base', 'exist(''CanBusObject'', ''var'')')
            elems(1) = Simulink.BusElement;
            elems(1).Name = 'rpm';
            elems(1).Dimensions = 1;
            elems(1).DataType = 'double';
    
            elems(2) = Simulink.BusElement;
            elems(2).Name = 'turns';
            elems(2).Dimensions = 1;
            elems(2).DataType = 'double';
    
            CanBusObject = Simulink.Bus;
            CanBusObject.Elements = elems;
            assignin('base', 'CanBusObject', CanBusObject);
        end
    
        %% 2. Add all blocks safely
        safe_add_block('simulink/Commonly Used Blocks/Constant', [modelName '/Distance Constant'], [100, 100, 150, 130], {});
    
        safe_add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/calculateLaunchParams'], [250, 90, 400, 140], {});
        
        safe_add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/velocityToRPM'], [550, 70, 700, 120], {});
        
        safe_add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/thetaToTurns'], [550, 150, 700, 200], {});
    
        safe_add_block('simulink/Sinks/Display', [modelName '/Display (RPM)'], [800, 70, 850, 120], {});
        
        safe_add_block('simulink/Sinks/Display', [modelName '/Display (Turns)'], [800, 150, 850, 200], {});
    
        safe_add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/canMessageFormat'], [1000, 90, 1150, 160], {});
        
        safe_add_block('simulink/Signal Routing/Bus Creator', [modelName '/BusCreator'], [1200, 90, 1250, 160], {});
        
        safe_add_block('simulink/Sinks/To Workspace', [modelName '/CAN Transmit'], [1350, 100, 1400, 140], {'VariableName', 'canTransmitSignal'});
    
        % 🔥 NEW: Add Trajectory Calculator
        safe_add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/trajectoryCalculator'], [250, 200, 400, 300], {});
        
        safe_add_block('simulink/Sinks/To Workspace', [modelName '/Trajectory Output'], [550, 250, 600, 290], {'VariableName', 'trajectoryData'});
    
        %% 3. Connect blocks safely
        try add_line(modelName, 'Distance Constant/1', 'calculateLaunchParams/1', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'calculateLaunchParams/1', 'velocityToRPM/1', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'calculateLaunchParams/2', 'thetaToTurns/1', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'velocityToRPM/1', 'Display (RPM)/1', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'thetaToTurns/1', 'Display (Turns)/1', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'velocityToRPM/1', 'canMessageFormat/1', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'thetaToTurns/1', 'canMessageFormat/2', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'canMessageFormat/1', 'BusCreator/1', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'canMessageFormat/2', 'BusCreator/2', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'BusCreator/1', 'CAN Transmit/1', 'autorouting', 'on'); catch; end
        
        % 🔥 NEW: Trajectory connections
        try add_line(modelName, 'calculateLaunchParams/1', 'trajectoryCalculator/1', 'autorouting', 'on'); catch; end
        try add_line(modelName, 'calculateLaunchParams/2', 'trajectoryCalculator/2', 'autorouting', 'on'); catch; end
        add_line(modelName, 'trajectoryCalculator/1', 'Trajectory Output/1', 'autorouting', 'on');
    
    
        %% 4. Save system
        save_system(modelName);
        disp(['✅ Model "' modelName '" updated and saved successfully!']);
    end
end
