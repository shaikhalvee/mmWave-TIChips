%--------------------------------------------------------------
% plot_range_pmm_diagram.m
% Loads PMM map, plots range vs PMM (averaged or max over frames/RX)
%--------------------------------------------------------------
adc_data_folder = 'D:\Documents\Drone_Data\data\tx_beamform_01\';  % update as needed
load(fullfile(adc_data_folder, 'PMMmap.mat'), 'PMMmap');
load(fullfile(adc_data_folder, 'radar_params_capture.mat'), 'params');

% Collapse PMMmap along frames and/or RX if desired
PMM_score = max(PMMmap, [], [2 3]);  % max over frames and RX

% Range axis (meters)
c = physconst('LightSpeed');
fs = params.Sampling_Rate_ksps * 1e3;
slope = params.Slope_MHzperus * 1e12;
range_axis = (0:size(PMMmap,1)-1) * c * fs / (2 * slope * size(PMMmap,1));

figure;
plot(range_axis, PMM_score, 'LineWidth', 2);
xlabel('Range (meters)'); ylabel('PMM Score');
title('Range-PMM Diagram');
grid on;
