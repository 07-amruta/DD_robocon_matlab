function robot_behavior = basketball_robot_database_system_updated_fixed()
    FIELD_LENGTH = 15.0;
    FIELD_WIDTH = 8.0;
    HOOP_HEIGHT = 2.43;
    PAINT_ZONE_SIDE = 2.5;
    PAINT_ZONE_RADIUS = 1.768;
    THREE_POINT_ARC_RADIUS = 3.1;
    ROBOT_HEIGHT = 2.0;
    IMAGE_WIDTH = 640;
    IMAGE_HEIGHT = 480;
    PIXEL_TO_METER_X = FIELD_LENGTH / IMAGE_WIDTH;
    PIXEL_TO_METER_Y = FIELD_WIDTH / IMAGE_HEIGHT;
    HOOP_PIXEL_X = 320;
    HOOP_PIXEL_Y = 100;
    HOOP_METER_X = 7.5;
    HOOP_METER_Y = 1.6667;
    
    fprintf('=== ABU Robocon 2025 Basketball Robot System - OFFICIAL Zone Dimensions ===\n');
    fprintf('Field Dimensions: %.1f x %.1f meters\n', FIELD_LENGTH, FIELD_WIDTH);
    fprintf('Hoop Height: %.2f meters\n', HOOP_HEIGHT);
    fprintf('Hoop Position: (%.1f, %.4f) meters\n', HOOP_METER_X, HOOP_METER_Y);
    fprintf('Paint Zone: %.1fm x %.1fm square box (7 points for dunk)\n', PAINT_ZONE_SIDE, PAINT_ZONE_SIDE);
    fprintf('2-Point Zone: Inside %.1fm radius arc (2 points)\n', THREE_POINT_ARC_RADIUS);
    fprintf('3-Point Zone: Outside %.1fm radius arc (3 points)\n', THREE_POINT_ARC_RADIUS);
    fprintf('Pixel to Meter Ratio: X=%.6f, Y=%.6f\n\n', PIXEL_TO_METER_X, PIXEL_TO_METER_Y);
    
    db_conn = setup_database_connection();
    total_frames = get_total_frames(db_conn);
    if total_frames == 0
        fprintf('Creating sample data with exact ABU Robocon 2025 specifications...\n');
        create_sample_detection_data_updated(PIXEL_TO_METER_X, PIXEL_TO_METER_Y);
        total_frames = get_total_frames(db_conn);
    end
    
    fprintf('Processing %d frames with updated specifications...\n\n', total_frames);
    behavior_analytics = initialize_analytics_tracking_fixed(total_frames);
    fig_analysis = setup_distance_speed_angle_visualization();
    frame_count = 1;
    successful_frames = 0;
    
    while frame_count <= total_frames
        try
            [hoop_data, basketball_data, timestamp] = get_detection_data(db_conn, frame_count);
            
            if ~isempty(hoop_data) && ~isempty(basketball_data)
                robot_behavior = calculate_database_behavior_corrected(hoop_data, basketball_data, ...
                    HOOP_HEIGHT, ROBOT_HEIGHT, PAINT_ZONE_RADIUS, THREE_POINT_ARC_RADIUS, ...
                    PIXEL_TO_METER_X, PIXEL_TO_METER_Y);
                
                behavior_analytics = update_analytics_data_corrected(behavior_analytics, ...
                    robot_behavior, hoop_data, basketball_data, frame_count, timestamp);
                
                update_distance_speed_angle_visualization_corrected(fig_analysis, behavior_analytics, ...
                    frame_count, PAINT_ZONE_RADIUS, THREE_POINT_ARC_RADIUS);
                
                log_robot_decision_corrected(robot_behavior, timestamp, frame_count);
                store_behavior_command(db_conn, robot_behavior, frame_count);
                successful_frames = successful_frames + 1;
            else
                robot_behavior = struct('action', 'search_mode', 'confidence', 0);
                fprintf('Frame %d: No detection data available\n', frame_count);
            end
            
        catch ME
            fprintf('Error processing frame %d: %s\n', frame_count, ME.message);
            robot_behavior = struct('action', 'error', 'confidence', 0);
        end
        
        frame_count = frame_count + 1;
        pause(0.15);
    end
    
    fprintf('\n✓ Processing complete: %d/%d frames successful\n', successful_frames, total_frames);
    generate_distance_speed_angle_analysis_corrected(behavior_analytics, HOOP_HEIGHT, PAINT_ZONE_RADIUS, THREE_POINT_ARC_RADIUS);
    close_database_connection(db_conn);
end

function analytics = initialize_analytics_tracking_fixed(total_frames)
    analytics = struct();
    analytics.frames = [];
    analytics.distances = [];
    analytics.angles = [];
    analytics.speeds = [];
    analytics.actions = {};
    analytics.points = [];
    analytics.confidences = [];
    analytics.hoop_positions = [];
    analytics.ball_positions = [];
    analytics.timestamps = {};
    analytics.horizontal_speeds = [];
    analytics.vertical_speeds = [];
    analytics.zone_classifications = {};
end

function analytics = update_analytics_data_corrected(analytics, robot_behavior, hoop_data, basketball_data, frame_count, timestamp)
    analytics.frames(end+1) = frame_count;
    analytics.distances(end+1) = robot_behavior.distance;
    analytics.actions{end+1} = robot_behavior.action;
    analytics.points(end+1) = robot_behavior.points_value;
    analytics.hoop_positions(end+1, :) = hoop_data.center;
    analytics.ball_positions(end+1, :) = basketball_data.center;
    analytics.timestamps{end+1} = timestamp;
    
    if isfield(robot_behavior, 'shooting_angle')
        analytics.angles(end+1) = robot_behavior.shooting_angle;
        analytics.speeds(end+1) = robot_behavior.shooting_speed;
        angle_rad = robot_behavior.shooting_angle * pi / 180;
        analytics.horizontal_speeds(end+1) = robot_behavior.shooting_speed * cos(angle_rad);
        analytics.vertical_speeds(end+1) = robot_behavior.shooting_speed * sin(angle_rad);
    else
        analytics.angles(end+1) = NaN;
        analytics.speeds(end+1) = NaN;
        analytics.horizontal_speeds(end+1) = NaN;
        analytics.vertical_speeds(end+1) = NaN;
    end
    
    if isfield(robot_behavior, 'confidence')
        analytics.confidences(end+1) = robot_behavior.confidence;
    else
        analytics.confidences(end+1) = NaN;
    end
    
    if robot_behavior.distance <= 1.768
        analytics.zone_classifications{end+1} = 'Paint Zone';
    elseif robot_behavior.distance <= 3.1
        analytics.zone_classifications{end+1} = '2-Point Zone';
    else
        analytics.zone_classifications{end+1} = '3-Point Zone';
    end
end

function fig = setup_distance_speed_angle_visualization()
    fig = figure('Name', 'ABU Robocon 2025 - Distance vs Speed vs Angle Analysis', ...
                 'Position', [100, 100, 1200, 800], 'Color', 'white');
    
    subplot(2, 3, 1); title('Distance from Hoop Over Time'); grid on; hold on;
    subplot(2, 3, 2); title('Required Shooting Speed vs Distance'); grid on; hold on;
    subplot(2, 3, 3); title('Optimal Shooting Angle vs Distance'); grid on; hold on;
    subplot(2, 3, 4); title('Speed Components Analysis'); grid on; hold on;
    subplot(2, 3, 5); title('Distance Distribution'); grid on; hold on;
    subplot(2, 3, 6); title('3D: Distance-Speed-Angle Relationship'); grid on; hold on;
    
    drawnow;
end

function update_distance_speed_angle_visualization_corrected(fig, analytics, current_frame, paint_zone_radius, three_point_arc_radius)
    figure(fig);
    
    if length(analytics.distances) < 2
        return;
    end
    
    subplot(2, 3, 1);
    cla;
    plot(analytics.frames, analytics.distances, 'b-o', 'LineWidth', 2, 'MarkerSize', 6);
    hold on;
    yline(paint_zone_radius, 'g--', 'LineWidth', 2, 'DisplayName', 'Paint Zone (1.768m)');
    yline(three_point_arc_radius, 'r--', 'LineWidth', 2, 'DisplayName', '3-Point Arc (3.1m)');
    xlabel('Frame Number'); ylabel('Distance (m)');
    title(sprintf('Distance from Hoop (Frame %d)', current_frame));
    legend('Distance', 'Paint Zone', '3-Point Arc', 'Location', 'best');
    grid on;
    
    subplot(2, 3, 2);
    cla;
    valid_speeds = ~isnan(analytics.speeds);
    if sum(valid_speeds) > 0
        scatter(analytics.distances(valid_speeds), analytics.speeds(valid_speeds), ...
               100, analytics.frames(valid_speeds), 'filled');
        colorbar; colormap(jet);
        
        dist_range = linspace(min(analytics.distances), max(analytics.distances), 50);
        speed_trend = calculate_theoretical_speed_trend(dist_range);
        hold on;
        plot(dist_range, speed_trend, 'r-', 'LineWidth', 3, 'DisplayName', 'Theoretical Trend');
        legend('Actual Data', 'Theoretical', 'Location', 'best');
    end
    xlabel('Distance from Hoop (m)'); ylabel('Required Speed (m/s)');
    title('Required Shooting Speed vs Distance');
    grid on;
    
    subplot(2, 3, 3);
    cla;
    valid_angles = ~isnan(analytics.angles);
    if sum(valid_angles) > 0
        scatter(analytics.distances(valid_angles), analytics.angles(valid_angles), ...
               100, analytics.points(valid_angles), 'filled');
        colorbar; colormap(hot);
        
        angle_trend = calculate_theoretical_angle_trend(dist_range);
        hold on;
        plot(dist_range, angle_trend, 'b-', 'LineWidth', 3, 'DisplayName', 'Optimal Angle');
        legend('Actual Data', 'Theoretical Optimal', 'Location', 'best');
    end
    xlabel('Distance from Hoop (m)'); ylabel('Shooting Angle (degrees)');
    title('Optimal Shooting Angle vs Distance');
    grid on;
    
    subplot(2, 3, 4);
    cla;
    valid_components = ~isnan(analytics.horizontal_speeds) & ~isnan(analytics.vertical_speeds);
    if sum(valid_components) > 0
        plot(analytics.distances(valid_components), analytics.horizontal_speeds(valid_components), ...
             'g-o', 'LineWidth', 2, 'DisplayName', 'Horizontal Speed');
        hold on;
        plot(analytics.distances(valid_components), analytics.vertical_speeds(valid_components), ...
             'm--s', 'LineWidth', 2, 'DisplayName', 'Vertical Speed');
    end
    xlabel('Distance from Hoop (m)'); ylabel('Speed Components (m/s)');
    title('Speed Components vs Distance');
    legend('Location', 'best');
    grid on;
    
    subplot(2, 3, 5);
    cla;
    histogram(analytics.distances, 8, 'FaceColor', [0.3, 0.7, 0.9], 'EdgeColor', 'black');
    hold on;
    xline(paint_zone_radius, 'g-', 'LineWidth', 3, 'DisplayName', 'Paint Zone');
    xline(three_point_arc_radius, 'r-', 'LineWidth', 3, 'DisplayName', '3-Point Arc');
    xlabel('Distance from Hoop (m)'); ylabel('Frequency');
    title('Distance Distribution');
    legend('Location', 'best');
    grid on;
    
    subplot(2, 3, 6);
    cla;
    valid_3d = ~isnan(analytics.angles) & ~isnan(analytics.speeds);
    if sum(valid_3d) > 0
        scatter3(analytics.distances(valid_3d), analytics.speeds(valid_3d), ...
                analytics.angles(valid_3d), 100, analytics.points(valid_3d), 'filled');
        colorbar; colormap(jet);
    end
    xlabel('Distance (m)'); ylabel('Speed (m/s)'); zlabel('Angle (degrees)');
    title('3D: Distance-Speed-Angle');
    grid on; view(45, 30);
    
    drawnow;
end

function speed_trend = calculate_theoretical_speed_trend(distances)
    hoop_height = 2.43;
    robot_height = 2.0;
    height_diff = hoop_height - robot_height;
    g = 9.8;
    
    base_angle = 46.9;
    distance_factor = atan(height_diff ./ distances) * 180 / pi;
    angles = base_angle + distance_factor * 0.3;
    angles = max(35, min(65, angles));
    
    angles_rad = angles * pi / 180;
    denominator = 2 * (cos(angles_rad).^2) .* (distances .* tan(angles_rad) - height_diff);
    
    speed_trend = sqrt(max(0, g * distances.^2 ./ max(0.1, denominator)));
    speed_trend = max(5, min(15, speed_trend));
end

function angle_trend = calculate_theoretical_angle_trend(distances)
    hoop_height = 2.43;
    robot_height = 2.0;
    height_diff = hoop_height - robot_height;
    
    base_angle = 46.9;
    distance_factor = atan(height_diff ./ distances) * 180 / pi;
    angle_trend = base_angle + distance_factor * 0.3;
    angle_trend = max(35, min(65, angle_trend));
end

function robot_behavior = calculate_database_behavior_corrected(hoop_data, basketball_data, ...
                                                               hoop_height, robot_height, ...
                                                               paint_zone_radius, three_point_arc_radius, ...
                                                               pixel_to_meter_x, pixel_to_meter_y)
    hoop_position_x = hoop_data.center(1) * pixel_to_meter_x;
    hoop_position_y = hoop_data.center(2) * pixel_to_meter_y;
    ball_position_x = basketball_data.center(1) * pixel_to_meter_x;
    ball_position_y = basketball_data.center(2) * pixel_to_meter_y;
    
    distance_to_hoop = sqrt((hoop_position_x - ball_position_x)^2 + ...
                           (hoop_position_y - ball_position_y)^2);
    
    if distance_to_hoop <= paint_zone_radius
        jump_height = calculate_required_jump_height(hoop_height, robot_height);
        
        robot_behavior = struct(...
            'action', 'attempt_dunk_shot', ...
            'distance', distance_to_hoop, ...
            'required_jump_height', jump_height, ...
            'points_value', 7, ...
            'dunk_feasibility', assess_dunk_feasibility(jump_height));
            
    elseif distance_to_hoop <= three_point_arc_radius
        optimal_angle = calculate_optimal_angle_exact(distance_to_hoop, hoop_height, robot_height);
        shooting_speed = calculate_shooting_velocity_exact(distance_to_hoop, hoop_height, robot_height, optimal_angle);
        
        robot_behavior = struct(...
            'action', 'execute_2pointer', ...
            'distance', distance_to_hoop, ...
            'shooting_angle', optimal_angle, ...
            'shooting_speed', shooting_speed, ...
            'points_value', 2, ...
            'confidence', calculate_shot_success_probability_exact(distance_to_hoop, optimal_angle));
            
    else
        optimal_angle = calculate_optimal_angle_exact(distance_to_hoop, hoop_height, robot_height);
        shooting_speed = calculate_shooting_velocity_exact(distance_to_hoop, hoop_height, robot_height, optimal_angle);
        
        robot_behavior = struct(...
            'action', 'execute_3pointer', ...
            'distance', distance_to_hoop, ...
            'shooting_angle', optimal_angle, ...
            'shooting_speed', shooting_speed, ...
            'points_value', 3, ...
            'shot_clock_remaining', 20, ...
            'confidence', calculate_shot_success_probability_exact(distance_to_hoop, optimal_angle));
    end
    
    robot_behavior.dribble_required = true;
    robot_behavior.possession_time = calculate_possession_duration();
end

function optimal_angle = calculate_optimal_angle_exact(distance, hoop_height, robot_height)
    height_difference = hoop_height - robot_height;
    base_angle = 46.9;
    distance_factor = atan(height_difference / distance) * 180 / pi;
    optimal_angle = base_angle + distance_factor * 0.3;
    optimal_angle = max(35, min(65, optimal_angle));
end

function shooting_speed = calculate_shooting_velocity_exact(distance, hoop_height, robot_height, angle_deg)
    angle_rad = angle_deg * pi / 180;
    height_diff = hoop_height - robot_height;
    gravity = 9.8;
    
    numerator = gravity * distance^2;
    denominator = 2 * cos(angle_rad)^2 * (distance * tan(angle_rad) - height_diff);
    
    if denominator > 0
        shooting_speed = sqrt(numerator / denominator);
    else
        shooting_speed = 8.0;
    end
    
    shooting_speed = max(5, min(15, shooting_speed));
end

function probability = calculate_shot_success_probability_exact(distance, angle)
    optimal_distance_3pt = 3.1;
    optimal_angle = 46.9;
    
    distance_factor = exp(-0.08 * abs(distance - optimal_distance_3pt));
    angle_factor = exp(-0.04 * abs(angle - optimal_angle));
    
    probability = distance_factor * angle_factor * 0.85;
    probability = max(0.1, min(0.85, probability));
end

function generate_distance_speed_angle_analysis_corrected(analytics, hoop_height, paint_zone_radius, three_point_arc_radius)
    fprintf('\n=== Generating ABU Robocon 2025 Distance-Speed-Angle Analysis ===\n');
    
    figure('Name', 'ABU Robocon 2025 Comprehensive Distance-Speed-Angle Analysis - OFFICIAL ZONES', ...
           'Position', [150, 150, 1200, 700]);
    
    subplot(2, 3, [1, 2]);
    valid_data = ~isnan(analytics.speeds);
    if sum(valid_data) > 0
        scatter(analytics.distances(valid_data), analytics.speeds(valid_data), ...
               150, analytics.angles(valid_data), 'filled', 'MarkerEdgeColor', 'black');
        colorbar; colormap(jet);
        
        dist_range = linspace(1, 8, 100);
        speed_theoretical = calculate_theoretical_speed_trend(dist_range);
        hold on;
        plot(dist_range, speed_theoretical, 'r-', 'LineWidth', 4, 'DisplayName', 'Theoretical (ABU Robocon 2025)');
        
        xline(paint_zone_radius, 'g--', 'LineWidth', 3, 'DisplayName', 'Paint Zone (1.768m)');
        xline(three_point_arc_radius, 'r--', 'LineWidth', 3, 'DisplayName', '3-Point Arc (3.1m)');
    end
    xlabel('Distance from Hoop (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Required Shooting Speed (m/s)', 'FontSize', 12, 'FontWeight', 'bold');
    title('Distance vs Required Speed (Color = Shooting Angle) - OFFICIAL ZONES', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best');
    grid on;
    
    subplot(2, 3, 3);
    if sum(valid_data) > 0
        scatter(analytics.distances(valid_data), analytics.angles(valid_data), ...
               100, analytics.confidences(valid_data), 'filled');
        colorbar; colormap(hot);
        
        angle_theoretical = calculate_theoretical_angle_trend(dist_range);
        hold on;
        plot(dist_range, angle_theoretical, 'b-', 'LineWidth', 3, 'DisplayName', 'Optimal Angle');
    end
    xlabel('Distance (m)'); ylabel('Shooting Angle (degrees)');
    title('Distance vs Optimal Angle');
    legend('Location', 'best');
    grid on;
    
    subplot(2, 3, 4);
    valid_components = ~isnan(analytics.horizontal_speeds);
    if sum(valid_components) > 0
        plot(analytics.distances(valid_components), analytics.horizontal_speeds(valid_components), ...
             'g-o', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Horizontal Component');
        hold on;
        plot(analytics.distances(valid_components), analytics.vertical_speeds(valid_components), ...
             'm-s', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Vertical Component');
        
        speed_theo = calculate_theoretical_speed_trend(dist_range);
        angle_theo = calculate_theoretical_angle_trend(dist_range);
        angle_theo_rad = angle_theo * pi / 180;
        plot(dist_range, speed_theo .* cos(angle_theo_rad), 'g--', 'LineWidth', 2, 'DisplayName', 'Theo. Horizontal');
        plot(dist_range, speed_theo .* sin(angle_theo_rad), 'm--', 'LineWidth', 2, 'DisplayName', 'Theo. Vertical');
    end
    xlabel('Distance (m)'); ylabel('Speed Components (m/s)');
    title('Speed Components Analysis');
    legend('Location', 'best');
    grid on;
    
    subplot(2, 3, 5);
    zone_names = unique(analytics.zone_classifications);
    zone_avg_speeds = zeros(size(zone_names));
    zone_avg_angles = zeros(size(zone_names));
    
    for i = 1:length(zone_names)
        zone_mask = strcmp(analytics.zone_classifications, zone_names{i});
        if sum(zone_mask) > 0
            zone_speeds = analytics.speeds(zone_mask);
            zone_angles = analytics.angles(zone_mask);
            zone_avg_speeds(i) = mean(zone_speeds(~isnan(zone_speeds)));
            zone_avg_angles(i) = mean(zone_angles(~isnan(zone_angles)));
        end
    end
    
    yyaxis left;
    bar_speeds = bar(zone_avg_speeds, 'FaceColor', [0.3, 0.7, 0.9]);
    ylabel('Average Speed (m/s)', 'Color', 'b');
    
    yyaxis right;
    hold on;
    bar_angles = bar(zone_avg_angles, 'FaceColor', [0.9, 0.5, 0.2], 'FaceAlpha', 0.7);
    ylabel('Average Angle (degrees)', 'Color', 'r');
    
    set(gca, 'XTickLabel', zone_names);
    xtickangle(45);
    title('Zone-wise Speed & Angle Analysis');
    grid on;
    
    subplot(2, 3, 6);
    summary_metrics = [
        mean(analytics.distances), ...
        mean(analytics.speeds, 'omitnan'), ...
        mean(analytics.angles, 'omitnan'), ...
        mean(analytics.confidences, 'omitnan') * 100, ...
        sum(analytics.points)
    ];
    
    metric_names = {'Avg Dist (m)', 'Avg Speed (m/s)', 'Avg Angle (°)', 'Avg Conf (%)', 'Total Points'};
    bar(summary_metrics, 'FaceColor', [0.6, 0.3, 0.8]);
    set(gca, 'XTickLabel', metric_names);
    xtickangle(45);
    title('Performance Summary');
    
    for i = 1:length(summary_metrics)
        text(i, summary_metrics(i) + max(summary_metrics)*0.02, ...
             sprintf('%.1f', summary_metrics(i)), ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    grid on;
    
    fprintf('✓ Distance-Speed-Angle analysis complete with OFFICIAL ABU Robocon 2025 zone specifications\n');
end

function create_sample_detection_data_updated(pixel_to_meter_x, pixel_to_meter_y)
    data_dir = './robocon_detection_data/';
    if ~exist(data_dir, 'dir')
        mkdir(data_dir);
    end
    
    for i = 1:20
        hoop_data = struct(...
            'center', [320 + rand()*20, 100 + rand()*20], ...
            'radius', 30 + rand()*10, ...
            'confidence', 0.85 + rand()*0.15);
        
        basketball_data = struct(...
            'center', [100 + rand()*440, 150 + rand()*280], ...
            'radius', 15 + rand()*8, ...
            'confidence', 0.75 + rand()*0.25);
        
        timestamp = datetime('now') + seconds(i);
        
        filename = sprintf('%sframe_%04d.mat', data_dir, i);
        save(filename, 'hoop_data', 'basketball_data', 'timestamp');
    end
    
    fprintf('✓ Created %d sample frames with exact ABU Robocon 2025 coordinates\n', 20);
end

function log_robot_decision_corrected(robot_behavior, timestamp, frame_id)
    fprintf('\n=== ABU Robocon 2025 Robot Decision - Frame %d ===\n', frame_id);
    fprintf('Timestamp: %s\n', string(timestamp));
    fprintf('Action: %s\n', robot_behavior.action);
    fprintf('Distance to Hoop: %.3f meters\n', robot_behavior.distance);
    
    if isfield(robot_behavior, 'shooting_angle')
        fprintf('Shooting Angle: %.2f degrees\n', robot_behavior.shooting_angle);
        fprintf('Shooting Speed: %.2f m/s\n', robot_behavior.shooting_speed);
    end
    
    fprintf('Points Value: %d points\n', robot_behavior.points_value);
    
    if isfield(robot_behavior, 'confidence')
        fprintf('Success Probability: %.1f%%\n', robot_behavior.confidence * 100);
    end
    
    if robot_behavior.distance <= 1.768
        fprintf('Zone: Paint Zone (≤1.768m) - 2.5m Square Box\n');
    elseif robot_behavior.distance <= 3.1
        fprintf('Zone: 2-Point Territory (1.768-3.1m) - Inside 3.1m Arc\n');
    else
        fprintf('Zone: 3-Point Territory (>3.1m) - Outside 3.1m Arc\n');
    end
    
    fprintf('Dribble Required: %s\n', string(robot_behavior.dribble_required));
    fprintf('===============================================\n');
end

function db_conn = setup_database_connection()
    try
        if license('test', 'Database_Toolbox')
            datasource = "ABU_Robocon_2025_Detection";
            db_conn = database(datasource, '', '', 'Vendor', 'MySQL', 'Server', 'localhost', 'PortNumber', 3306, 'LoginTimeout', 5);
            if isempty(db_conn.Message)
                fprintf('✓ Connected to ABU Robocon 2025 Detection Database\n');
                return;
            else
                error('Database connection failed: %s', db_conn.Message);
            end
        else
            error('Database Toolbox not available');
        end
    catch ME
        fprintf('Database connection failed: %s\n', ME.message);
        fprintf('Using local data files instead...\n');
        db_conn = setup_local_data_connection();
    end
end

function db_conn = setup_local_data_connection()
    db_conn = struct('type', 'local', 'data_path', './robocon_detection_data/');
    if ~exist(db_conn.data_path, 'dir')
        mkdir(db_conn.data_path);
        fprintf('✓ Created local data directory: %s\n', db_conn.data_path);
    end
    fprintf('✓ Using local file system for data storage\n');
end

function [hoop_data, basketball_data, timestamp] = get_detection_data(db_conn, frame_id)
    if isstruct(db_conn) && isfield(db_conn, 'type') && strcmp(db_conn.type, 'local')
        [hoop_data, basketball_data, timestamp] = load_local_detection_data(frame_id);
    else
        try
            hoop_query = sprintf(['SELECT hoop_center_x, hoop_center_y, hoop_radius, ' ...
                                 'confidence, detection_timestamp FROM hoop_detections ' ...
                                 'WHERE frame_id = %d'], frame_id);
            basketball_query = sprintf(['SELECT ball_center_x, ball_center_y, ball_radius, ' ...
                                       'confidence, detection_timestamp FROM basketball_detections ' ...
                                       'WHERE frame_id = %d'], frame_id);
            
            hoop_result = select(db_conn, hoop_query);
            basketball_result = select(db_conn, basketball_query);
            
            if height(hoop_result) > 0
                hoop_data = struct('center', [hoop_result.hoop_center_x(1), hoop_result.hoop_center_y(1)], 'radius', hoop_result.hoop_radius(1), 'confidence', hoop_result.confidence(1));
                timestamp = hoop_result.detection_timestamp{1};
            else
                hoop_data = []; timestamp = datetime('now');
            end
            
            if height(basketball_result) > 0
                basketball_data = struct('center', [basketball_result.ball_center_x(1), basketball_result.ball_center_y(1)], 'radius', basketball_result.ball_radius(1), 'confidence', basketball_result.confidence(1));
            else
                basketball_data = [];
            end
        catch ME
            fprintf('Database query error: %s\n', ME.message);
            hoop_data = []; basketball_data = []; timestamp = datetime('now');
        end
    end
end

function [hoop_data, basketball_data, timestamp] = load_local_detection_data(frame_id)
    data_file = sprintf('./robocon_detection_data/frame_%04d.mat', frame_id);
    if exist(data_file, 'file')
        loaded_data = load(data_file);
        hoop_data = loaded_data.hoop_data;
        basketball_data = loaded_data.basketball_data;
        timestamp = loaded_data.timestamp;
    else
        hoop_data = []; basketball_data = []; timestamp = datetime('now');
    end
end

function duration = calculate_possession_duration()
    duration = rand() * 5;
end

function feasibility = assess_dunk_feasibility(jump_height)
    max_robot_jump = 0.5;
    feasibility = jump_height <= max_robot_jump;
end

function jump_height = calculate_required_jump_height(hoop_height, robot_height)
    basket_rim_height = hoop_height;
    robot_reach_height = robot_height + 0.3;
    jump_height = max(0, basket_rim_height - robot_reach_height);
    jump_height = jump_height + 0.1;
end

function store_behavior_command(db_conn, robot_behavior, frame_id)
    if isstruct(db_conn) && isfield(db_conn, 'type') && strcmp(db_conn.type, 'local')
        behavior_dir = './abu_robocon_2025_behaviors/';
        if ~exist(behavior_dir, 'dir')
            mkdir(behavior_dir);
        end
        filename = sprintf('%sbehavior_%04d.mat', behavior_dir, frame_id);
        save(filename, 'robot_behavior');
    else
        try
            insert_query = sprintf(['INSERT INTO robot_commands (frame_id, action, ' ...
                                   'distance, angle, speed, points_value, confidence, timestamp) ' ...
                                   'VALUES (%d, ''%s'', %.3f, %.2f, %.2f, %d, %.3f, NOW())'], ...
                                   frame_id, robot_behavior.action, robot_behavior.distance, ...
                                   robot_behavior.shooting_angle, robot_behavior.shooting_speed, ...
                                   robot_behavior.points_value, robot_behavior.confidence);
            exec(db_conn, insert_query);
        catch ME
            fprintf('Failed to store behavior command: %s\n', ME.message);
        end
    end
end

function total_frames = get_total_frames(db_conn)
    if isstruct(db_conn) && isfield(db_conn, 'type') && strcmp(db_conn.type, 'local')
        files = dir('./robocon_detection_data/frame_*.mat');
        total_frames = length(files);
    else
        try
            result = select(db_conn, 'SELECT MAX(frame_id) as max_frame FROM hoop_detections');
            total_frames = result.max_frame(1);
        catch
            total_frames = 0;
        end
    end
end

function close_database_connection(db_conn)
    if ~(isstruct(db_conn) && isfield(db_conn, 'type') && strcmp(db_conn.type, 'local'))
        try
            close(db_conn);
            fprintf('✓ Database connection closed\n');
        catch
            fprintf('Database connection already closed\n');
        end
    else
        fprintf('✓ Local file system connection closed\n');
    end
end
