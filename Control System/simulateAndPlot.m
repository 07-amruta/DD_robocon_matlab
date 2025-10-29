if ~bdIsLoaded('ShootingControlSystem_working')
    build_simulink_model();
end

distanceBlockPath = 'ShootingControlSystem_working/Distance Constant';
distanceValue = str2double(get_param(distanceBlockPath, 'Value'));

simOut = sim('ShootingControlSystem_working');

trajectoryData = simOut.get('trajectoryData');

data = trajectoryData.Data(:,:,1);

if isempty(data) || size(data, 1) < 2
    error('Trajectory data is missing or invalid.');
end

xData = data(1,:);
yData = data(2,:);

validIdx = (xData > 0 | yData > 0) & ~isnan(xData) & ~isnan(yData);

changes = diff([0, validIdx, 0]);
startIndices = find(changes == 1);
endIndices = find(changes == -1) - 1;
segments = cell(length(startIndices), 1);

figure;
hold on;
for i = 1:length(startIndices)
    segmentX = xData(startIndices(i):endIndices(i));
    segmentY = yData(startIndices(i):endIndices(i));
    plot(segmentX, segmentY, 'b-', 'LineWidth', 2);
end

xlabel('Distance (m)', 'FontSize', 12);
ylabel('Height (m)', 'FontSize', 12);
title('Ball Trajectory', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

xlim([0, max(max(xData(validIdx)), 10)]); 
ylim([0, max(max(yData(validIdx)), 10)]); 

distance = distanceValue; 
hoopHeight = 2.43; 

hoopWidth = 0.5;
hoopThickness = 0.05; 

plot([distance distance], [0 hoopHeight + 0.5], 'k-', 'LineWidth', 2);

rectangle('Position', [distance - hoopWidth/2, hoopHeight - hoopThickness/2, hoopWidth, hoopThickness], ...
    'Curvature', [1, 1], ... 
    'EdgeColor', 'r', ...
    'LineWidth', 3);

hold off;
disp('✅ Trajectory plotted with basketball hoop (unwanted line removed).');