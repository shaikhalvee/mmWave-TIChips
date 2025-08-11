%---------------------------------------------------------------
% SSTFRFT on FMCW slow-time (per range, angle) using OMP
% Inputs: IF data folder (same as your pipeline), params, calib
% Output: SSTFRFT coefficient cubes + time–freq maps per frame
%---------------------------------------------------------------
close all; clc;

%% ---------------- USER CONFIG --------------------------------
adc_data_folder = 'G:\RADAR_DATA\out_txbf_6_8_sl_22_fr_200';
[~, testRootFolder, ~] = fileparts(adc_data_folder);
output_folder = ['./output/' testRootFolder];
oldParamsFile = [output_folder filesep testRootFolder '_params.mat'];
calib_file = './input/calibrConfig/calibrateResults_dummy.mat';

% Which angle index to process (camera-guided)
angle_idx = 1;                      % <-- set or pass in dynamically

% Range bins to analyze (avoid whole cube for speed)
rng_bins_to_process = 50:300;       % <-- set this based on your scene

% SSTFRFT config (defaults—tune later)
cfg.win_len_chirps = 64;            % window length in chirps
cfg.hop_chirps     = 32;            % hop in chirps
cfg.K              = 6;             % OMP sparsity per window
cfg.use_dc_hp      = true;          % slow-time mean removal per cell

% Grids (slow-time frequency & chirp-rate)
cfg.f0_hz   = [];                   % [] => auto from PRF: [-PRF/2, +PRF/2]
cfg.Nf      = 128;                  % #freq atoms if auto
cfg.mu_hz_s = [];                   % [] => symmetric small accel grid
cfg.Nmu     = 25;                   % #chirp-rate atoms if auto. 25, a middle number or a balance point
cfg.mu_frac_of_f0span = 0.15;       % accel grid width ~ 15% of f0 span

% Optional: Rx combining
do_rx_beamform = false;                % if false: phase-cal + sum over Rx

%% --------------- LOAD PARAMS & CALIB -------------------------
S = load(oldParamsFile, 'params'); 
params = S.params;
load(calib_file, 'calibResult');    % BF_MIMO_ref (deg per Rx)
BF_MIMO_ref = calibResult.RxMismatch;
params.BF_MIMO_ref = BF_MIMO_ref;

%% DC clutter removal
dcOffsetRemoval = true;

% derive PRF / PRT / lambda
c = physconst('LightSpeed');
PRT = (params.Idle_Time_us + params.Ramp_End_Time_us)*1e-6; % Chirp duration
params.PRT = PRT;
params.PRF = 1/PRT; % chirp frequency
params.lambda = c/(params.Start_Freq_GHz*1e9);

%% --------------- READ FILE INDEXES ---------------------------
[fileIdx_unique] = getUniqueFileIdx(adc_data_folder);

% storage folders
sst_out_folder = fullfile(output_folder,'sstfrft');
if ~isfolder(sst_out_folder) 
    mkdir(sst_out_folder); 
end

frameCounter = 1;
for i_file = 1:numel(fileIdx_unique)
    fns = getBinFileNames_withIdx(adc_data_folder, fileIdx_unique{i_file});
    [numValidFrames, ~] = getValidNumFrames(fullfile(adc_data_folder, fns.masterIdxFile));

    for frameId = 2:numValidFrames
        params.frameId = frameId;

        % ---------- LOAD RAW ADC (all RX, all angles) -------------
        adcRadarData_txbf = fcn_read_AdvFrmConfig_BF_Json(fns, params);
        adcRadarData_txbf = adcRadarData_txbf(:,:,params.TI_Cascade_RX_ID,:); % select good RXs

        % ---------- RANGE FFT → slow-time cube --------------------
        [range_fft_cube, range_axis, params] = build_range_only_cube(adcRadarData_txbf, params, dcOffsetRemoval);
        % rng_fft_cube: [R, K, Rx, Ang]

        % ---------- RX combine at chosen angle --------------------
        % TODO: need to work on angles later
        x_rka = range_fft_cube(:,:, :, angle_idx);           % [R,K,Rx,1]
        x_rka = squeeze(x_rka);                              % [R,K,Rx]
        
        % apply RX phase calib
        for rx=1:size(x_rka,3)
            x_rka(:,:,rx) = x_rka(:,:,rx) * exp(-1j*params.BF_MIMO_ref(rx)*pi/180);
        end

        if do_rx_beamform
            % TODO: replace a(theta0) with the steering vector if desired
            % technically not beamform, but rather signal enhancement,
            % possibly music along antenna dimension
            error('Beamforming weights not provided. Set do_beamform=false or supply a(theta0).');
        else
            % calibrated sum over Rx (keeps SNR gain, simple)
            x_rk = sum(x_rka,3); % [R,K]
        end

        % ---------- select ranges of interest ---------------------
        x_rk = x_rk(rng_bins_to_process, :);
        r_axis_sel = range_axis(rng_bins_to_process);

        % ---------- run SSTFRFT per range bin ---------------------
        cfg_local = sstfrft_autogrids(cfg, params.PRF, size(x_rk,2));
        [C_cell, tf_maps_cell, meta] = sstfrft_run_on_ranges(x_rk, params.PRF, cfg_local);

        % save outputs for this frame
        save(fullfile(sst_out_folder, sprintf('sstfrft_frame_%05d.mat', frameCounter)), ...
             'C_cell','tf_maps_cell','r_axis_sel','cfg_local','meta','-v7.3');

        fprintf('[SSTFRFT] frame %d done (%d ranges, angle %d)\n', ...
                 frameCounter, numel(rng_bins_to_process), angle_idx);
        frameCounter = frameCounter + 1;
    end
end
