% Setup (same as your viewer)
data_folder  = './output/out_txbf_sim_uav_3p5ms2/';
frame_folder = [data_folder 'rangeDopplerFFTmap_sim/'];

frame_files = dir(fullfile(frame_folder, 'frame_*.mat'));
% config_data = load(fullfile(data_folder, 'config.mat'));
% all_range_axis   = config_data.all_range_axis;
% all_doppler_axis = config_data.all_doppler_axis;



% ===================== LOAD PARAMS & BUILD AXES =====================
params_folder = data_folder;
params_file = dir(fullfile(data_folder, '*_params.mat'));
assert(~isempty(params_file), 'Cannot find *_params.mat in %s', params_folder);
tmp = load(fullfile(params_folder, params_file(1).name), 'params');
params = tmp.params;

% ===================== Range & Doppler axis calculation =============
numOfFrames = numel(frame_files);
range_axis = (0:params.rangeFFTSize-1) * c * fs / (2 * params.slope * params.rangeFFTSize);
all_range_axis = repmat(range_axis(:), 1, numOfFrames);
doppler_axis = linspace(-params.v_max, params.v_max, params.dopplerFFTSize);
all_doppler_axis = repmat(doppler_axis(:), 1, numOfFrames);

maxRange = 100;   % same as viewer

% Gate options (match your viewer)
droneOpts.auto_gate  = true;
droneOpts.gate_center_m = 80;
droneOpts.gate_width_m = 5;
droneOpts.v_exclude = 0.5; % m/s
% droneOpts.v_res = params.velocityResolution;
droneOpts.v_max = params.v_max;

% Folding parameters & angle
jmin = 2;
jmax = 32; 
angleIdx = 1;

% Run correlation analysis
stats = analyze_corr_concentration_accel( ...
    frame_files, all_range_axis, all_doppler_axis, ...
    jmin, jmax, maxRange, droneOpts, angleIdx);

fprintf('dataset: %s\n', frame_folder);
fprintf('Spearman rho(C*, |Δv|)  = %.3f\n', stats.rhoS_C_aMag);
fprintf('Spearman rho(ΔC, |Δv|)  = %.3f\n', stats.rhoS_dC_aMag);
% fprintf('Spearman rho(C*, Δv)    = %.3f\n', stats.rhoS_Cdv);
% fprintf('Spearman rho(C*, v)     = %.3f\n', stats.rhoS_Cv);
% fprintf('Pearson  r(C*, v)       = %.3f\n', stats.Rlin_Cv);
% fprintf('Pearson  r(C*, Δv)      = %.3f\n', stats.Rlin_Cdv);



% 1) C* vs velocity
validCv = ~isnan(stats.C_star) & ~isnan(stats.v_star);
figure;
scatter(stats.v_star(validCv), stats.C_star(validCv), 'filled');
xlabel('Velocity v (m/s)');
ylabel('Concentration C^*');
title(sprintf('C^* vs v  (Spearman \\rho = %.3f)', stats.rhoS_Cv));
grid on;

% 2) C* vs |Δv|
C2   = stats.C_star(2:end);
aMag = stats.aMag;
validCa = ~isnan(C2) & ~isnan(aMag);
figure;
scatter(aMag(validCa), C2(validCa), 'filled');
xlabel('|\\Delta v| (m/s)');
ylabel('Concentration C^*(f)');
title(sprintf('C^* vs |\\Delta v|  (Spearman \\rho = %.3f)', stats.rhoS_C_aMag));
grid on;

