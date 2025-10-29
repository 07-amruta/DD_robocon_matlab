function runRoboconWithSimulink()
    
    disp('🔧 Setting up Simulink model...');
    create_omni_shooter_model_final();
    modelName = 'omni_shooter_model_final';
    
    set_param(modelName, 'StopTime', 'inf');
    set_param(modelName, 'FixedStep', '0.05');
    set_param(modelName, 'Solver', 'FixedStepDiscrete');

    disp('🚀 Starting Simulink simulation...');
    set_param(modelName, 'SimulationCommand', 'start');
    
    disp('🎮 Starting Robocon gameplay with Simulink integration...');
    robocon3DGameplaySimulation_WithSimulink();
    
    if strcmp(get_param(modelName, 'SimulationStatus'), 'running')
        set_param(modelName, 'SimulationCommand', 'stop');
    end
    disp('✅ Simulation completed!');
end
