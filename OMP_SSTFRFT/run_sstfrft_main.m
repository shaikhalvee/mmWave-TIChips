%---------------------------------------------------------------
% SSTFRFT on FMCW slow-time (per range, angle) using OMP
% Inputs: IF data folder (same as your pipeline), params, calib
% Output: SSTFRFT coefficient cubes + time–freq maps per frame
%---------------------------------------------------------------
close all; clc;

%% ---------------- USER CONFIG --------------------------------
adc_data_folder = 'G:\RADAR_DATA\out_txbf_25_8_sl_15_fr_200';
[~, testRootFolder, ~] = fileparts(adc_data_folder);
output_folder = ['./output/' testRootFolder];
oldParamsFile = [output_folder filesep testRootFolder '_params.mat'];
calib_file = './input/calibrConfig/calibrateResults_dummy.mat';

% Which angle  index to process (camera-guided)
% angle_idx = 9;                      % <-- set or pass in dynamically

% Range bins to analyze (avoid whole cube for speed)
rng_bins_to_process = 1:512;       % <-- set this based on your scene

% SSTFRFT config (defaults—tune later)
cfg.win_len_chirps = 64;            % window length in chirps
cfg.hop_chirps     = 32;            % hop in chirps
cfg.K              = 6;             % OMP sparsity per window
cfg.use_dc_hp      = false;          % slow-time mean removal per cell

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
numAngle = params.NumAnglesToSweep;

%% DC clutter removal
dcOffsetRemoval = true;

% derive PRF / PRT / lambda
c = physconst('LightSpeed');
PRT = (params.Idle_Time_us + params.Ramp_End_Time_us)*1e-6; % Chirp duration
params.PRT = PRT;
params.PRF = 1/PRT; % chirp frequency
params.lambda = c/(params.Start_Freq_GHz*1e9);

% Doppler FFT size (match your RD)
params.dopplerFFTSize = 2^nextpow2(params.nchirp_loops);

%% --------------- READ FILE INDEXES ---------------------------
[fileIdx_unique] = getUniqueFileIdx(adc_data_folder);

% storage folders
% sst_out_folder = fullfile(output_folder,'sstfrft');
% if ~isfolder(sst_out_folder) 
%     mkdir(sst_out_folder);
% end

rd_out_folder = fullfile(output_folder, 'RD_SST');
if ~isfolder(rd_out_folder)
    mkdir(rd_out_folder); 
end


frameCounter = 1;
for i_file = 1:numel(fileIdx_unique)
    fileNameStruct = getBinFileNames_withIdx(adc_data_folder, fileIdx_unique{i_file});
    [numValidFrames, ~] = getValidNumFrames(fullfile(adc_data_folder, fileNameStruct.masterIdxFile));

    for frameId = 1:numValidFrames
        params.frameId = frameId;

        A_cell = {};
        C_cell = {};

        % ---------- LOAD RAW ADC (all RX, all angles) -------------
        adcRadarData_txbf = fcn_read_AdvFrmConfig_BF_Json(fileNameStruct, params);
        adcRadarData_txbf = adcRadarData_txbf(:,:,params.TI_Cascade_RX_ID,:); % select good RXs

        % ---------- RANGE FFT → slow-time cube --------------------
        [range_fft_cube, range_axis, params] = build_range_only_cube(adcRadarData_txbf, params, dcOffsetRemoval);
        % range_fft_cube: [R, K, Rx, Ang]
        Rtot = size(range_fft_cube,1);
        rng_sel = intersect(rng_bins_to_process, 1:Rtot);
        range_axis = range_axis(rng_sel);

        
        % ---------- LOOP ALL ANGLES -------------------------------
        for angle_idx = 1:numAngle
            % Extract one angle, phase-calibrate RX, combine
            x_rka = squeeze(range_fft_cube(:,:, :, angle_idx));     % [R,K,Rx]
            
            for rx = 1:size(x_rka,3)
                x_rka(:,:,rx) = x_rka(:,:,rx) * exp(-1j*params.BF_MIMO_ref(rx)*pi/180);
            end
            
            % RX calibration
            if do_rx_beamform
                error('Beamforming weights not provided. Set do_rx_beamform=false or supply weights.');
            else
                x_rk = mean(x_rka,3);                                % [R,K]
            end

            % Range selection
            x_rk = x_rk(rng_sel, :);                                % [Rsel,K]

            % ---------- SSTFRFT per range bin ---------------------
            cfg_local = sstfrft_autogrids(cfg, params.PRF, size(x_rk,2));
            [A_cell_per_ang, C_cell_per_ang, tf_maps_cell_per_ang, meta] = sstfrft_run_on_ranges(x_rk, params.PRF, cfg_local);

            % A_cell(:,:, angle_idx) = (A_cell_per_ang);
            % C_cell(:,:, angle_idx) = (C_cell_per_ang);

            % ---------- Reconstruct slow-time → RD FFT ------------
            [RD_sst, doppler_axis] = reconstruct_RD_from_sstfrft(A_cell_per_ang, meta, params);

            % Save RD (per angle)
            RD_map(:,:,angle_idx) = RD_sst;  % keep name consistent with your RD files if you like
            
        end
  
        % Save SSTFRFT products (per angle)
        % save(fullfile(sst_out_folder, sprintf('sstfrft_frame_%05d.mat', frameCounter)), ...
                % 'A_cell','C_cell','range_axis','cfg_local','meta','-v7.3');
        save(fullfile(rd_out_folder, ...
                sprintf('frame_%05d.mat', frameCounter)), 'RD_map','range_axis','doppler_axis','-v7.3');

        fprintf('[Frame %d] saved RD_SST (R=%d, D=%d, Ang=%d)\n', ...
            frameCounter, size(RD_map,1), size(RD_map,2), size(RD_map,3) );

        frameCounter = frameCounter + 1;
    end 
end
