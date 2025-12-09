function sim_uav_generate_frames()
%SIM_UAV_GENERATE_FRAMES
% Generate simulated UAV RD frames (with acceleration + micro-Doppler)
% and save them in a format directly readable by
% interactive_txbf_viewer_filtered.m.
%
% REQUIREMENTS:
%   - simulate_uav_rd_mmwcas.m on MATLAB path
%   - uav_rd_sim.m is NOT required (we inline the radar/UAV params here)
%
% After running this function:
%   data_folder  = fullfile('output', root_name);
%   frame_folder = fullfile(data_folder, frame_folder_name);
% and point interactive_txbf_viewer_filtered.m to these.

    %% ---------------- USER CONFIG ----------------
    % Global frame timing / trajectory
    T_frame  = 127.5e-3;   % s, frame periodicity
    a_mag    = 3.5;        % m/s^2, magnitude of radial accel
    t_accel  = 5.0;        % s, accelerate for 5 s, then decel 5 s
    T_total  = 12.0;       % s, total simulation duration (>= 2*t_accel)
    R0_init  = 10.0;       % m, initial range
    v0_init  = 0.0;        % m/s, initial radial velocity

    % Output folder / naming (mimic your txbf pipeline)
    root_name         = 'out_txbf_sim_uav_3p5ms2';
    data_folder       = fullfile('output', root_name);
    frame_folder_name = 'rangeDopplerFFTmap_sim';
    frame_folder      = fullfile(data_folder, frame_folder_name);

    % Create folders if needed
    if ~exist(data_folder, 'dir')
        mkdir(data_folder);
    end
    if ~exist(frame_folder, 'dir')
        mkdir(frame_folder);
    end

    %% ---------------- RADAR PARAMETERS ----------------
    % These are copied from your uav_rd_sim.txt as defaults
    c = physconst('LightSpeed');

    radar.fc = 77e9;                          % Hz
    radar.lambda = c / radar.fc;

    radar.N_samp = 512;                       % ADC samples per chirp
    radar.Fs     = 16e6;                      % ADC sampling rate [Hz]

    radar.S_slope_in_mhzus = 15.0148;         % MHz/us, from your board
    radar.T_chirp          = 250e-6;          % s, chirp duration
    radar.N_chirps         = 256;             % chirps per CPI

    % Derived params
    radar.Fslow    = 1 / radar.T_chirp;       % slow-time sampling rate
    radar.T_ch_eff = radar.N_samp / radar.Fs; % effective sweep time
    radar.S_slope  = radar.S_slope_in_mhzus * 1e12; % Hz/s
    radar.BW       = radar.S_slope * radar.T_ch_eff;

    radar.Nfft_range   = radar.N_samp;
    radar.Nfft_doppler = radar.N_chirps;

    radar.R_res = c / (2 * radar.BW);
    radar.R_max = c * radar.Fs / (2 * radar.S_slope);

    T_cpi       = radar.N_chirps * radar.T_chirp;
    radar.v_res = radar.lambda / (2 * T_cpi);
    radar.v_max = radar.lambda / (4 * radar.T_chirp);

    fprintf('Radar: Fc = %.2f GHz, BW = %.1f MHz\n', radar.fc/1e9, radar.BW/1e6);
    fprintf('  Range res = %.3f m, R_max = %.1f m\n', radar.R_res, radar.R_max);
    fprintf('  Doppler res = %.3f m/s, v_max = ±%.2f m/s\n', ...
            radar.v_res, radar.v_max);

    %% ---------------- UAV MICRO-DOPPLER MODEL ----------------
    % These match your uav_rd_sim defaults (except R0, v0, a which we
    % update per-frame).
    uav = struct();

    % Angle between blade plane and LOS
    uav.beta_deg = 0;
    uav.beta     = deg2rad(uav.beta_deg);

    % Body scattering
    uav.N_body_scatter = 50;
    uav.body_rcs       = 0.5;

    % Rotors / blades
    uav.N_rotors         = 4;     % quadcopter
    uav.N_blades_per_rot = 4;
    uav.N_scat_per_blade = 10;
    uav.blade_length     = 0.12;  % m
    uav.uav_rot_Hz       = 40;    % rotor rev frequency ~ 2400 rpm
    uav.blade_rcs        = 0.25;   % blades total RCS weight

    % Small rotor-to-rotor variation to smear the micro-Doppler lines
    uav.rotor_freq_jitter = 0.05; % ±0% no need for this. unnecessary

    % Noise model
    noise.SNR_dB = 20;            % dB SNR of beat signal

    %% ---------------- FRAME LOOP SETUP ----------------
    % Number of frames to simulate
    N_frames  = floor(T_total / T_frame) + 1;
    frame_ids = 1:N_frames;

    fprintf('\nSimulating %d frames, T_frame = %.3f s, total time ≈ %.2f s\n', ...
            N_frames, T_frame, (N_frames-1)*T_frame);

    %% ---------------- MAIN FRAME LOOP ----------------
    for k = 1:N_frames
        t_frame = (k-1) * T_frame;

        % Global 1D motion: accelerate, then decelerate, then hover
        [R0_k, v0_k, a_k] = uav_motion_profile(t_frame, R0_init, v0_init, a_mag, t_accel);

        % Plug into UAV struct for this CPI
        uav_k      = uav;
        uav_k.R0   = R0_k;
        uav_k.v0   = v0_k;
        uav_k.a    = a_k;

        % ---- This call includes:
        %   - Translational Doppler + ACCELERATION -> Doppler smear
        %   - Blade micromotion -> micro-Doppler lines around body
        [~, ~, ~, ~, aux] = simulate_uav_rd_mmwcas(radar, uav_k, noise);

        % aux.RD_cube is complex RD map: [Nfft_range x Nfft_doppler]
        RD_cube = aux.RD_cube;

        % Wrap into RD_map struct the viewer expects
        RD_map = struct();
        RD_map.dopplerFFT = RD_cube;   % [R x D], viewer will reshape to [R x D x 1]
        RD_map.rangeFFT = aux.S_range;

        fname = fullfile(frame_folder, sprintf('frame_%05d.mat', frame_ids(k)));
        save(fname, 'RD_map', '-v7.3');

        if mod(k,10) == 0 || k == 1 || k == N_frames
            fprintf('  Saved frame %5d at t = %.3f s: R0 = %.2f m, v0 = %.2f m/s, a = %.2f m/s^2\n', ...
                    frame_ids(k), t_frame, R0_k, v0_k, a_k);
        end
    end

    %% ---------------- PARAMS STRUCT FOR VIEWER ----------------
    params = struct();
    params.fs            = radar.Fs;
    params.rangeFFTSize  = radar.Nfft_range;
    params.slope         = radar.S_slope;
    params.dopplerFFTSize= radar.Nfft_doppler;
    params.v_max         = radar.v_max;

    % Single "beam" so the angle slider is automatically disabled
    params.anglesToSteer    = 0;    % 1-element vector
    params.NumAnglesToSweep = 1;

    % Extra info (not strictly required by viewer but useful)
    params.lambda        = radar.lambda;
    params.fc            = radar.fc;
    params.c             = c;
    params.R_res         = radar.R_res;
    params.R_max         = radar.R_max;
    params.v_res         = radar.v_res;
    params.framePeriod_s = T_frame;
    params.total_frames  = N_frames;
    params.frame_folder  = frame_folder_name;

    params_file = fullfile(data_folder, [root_name '_params.mat']);
    save(params_file, 'params');

    fprintf('\nSaved params file: %s\n', params_file);
    fprintf('Frames are in: %s\n', frame_folder);
    fprintf('Point interactive_txbf_viewer_filtered.m to data_folder = ''%s''\n', data_folder);

end
