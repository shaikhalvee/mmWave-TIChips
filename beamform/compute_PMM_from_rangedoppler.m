%--------------------------------------------------------------
% compute_PMM_from_rangedoppler.m
% Loads range-Doppler FFT data, computes PMM map, saves result
%--------------------------------------------------------------
adc_data_folder = 'D:\Documents\Drone_Data\data\tx_beamform_01\';  % update as needed
load(fullfile(adc_data_folder, 'rangeDopplerFFTData.mat'), 'fft_complex_radar_cube');

jMin = 2; jMax = 20;
PMMmap = calc_PMM_beamform(fft_complex_radar_cube, jMin, jMax);

save(fullfile(adc_data_folder, 'PMMmap.mat'), 'PMMmap');
fprintf('[INFO] Saved PMM map to %s\n', fullfile(adc_data_folder, 'PMMmap.mat'));
