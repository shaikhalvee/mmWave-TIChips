%% UAV Range–Doppler Simulator (FMCW, micro-Doppler + Doppler smearing)
% This script:
%   1) defines radar & UAV parameters,
%   2) simulates the complex FMCW beat signal,
%   3) computes the range–Doppler map,
%   4) plots the RD heatmap.

clear; clc; close all;

%% ------------- 1. RADAR PARAMETERS ------------------------------------
c = physconst('LightSpeed');            % speed of light (m/s)

radar.fc = 77e9;                  % carrier (Hz)
radar.lambda = c / radar.fc;
radar.N_samp = 512;                     % ADC samples per chirp (fast-time)
radar.Fs = 16e6;                        % fast-time sampling rate or ADC sampling rate
radar.S_slope_in_mhzus = 15.0148;    % Chirp slope S (MHz/us)
% radar.T_idle = 150e-6;
% radar.T_ramp_end = 100e-6;
% radar.T_chirp = radar.T_idle + radar.T_ramp_end;  % will implement later.
% I need to see the relation of idle time with the UAV signal
radar.T_chirp  = 250e-6;                % chirp duration (s)  (short to capture rotor MD)
radar.N_chirps = 256;                   % number of chirps in CPI (slow-time)

%% Derived parameters
% radar.S_slope  = radar.BW / radar.T_chirp;        % chirp slope S (Hz/s)
% radar.Fs       = radar.N_samp / radar.T_chirp;    % fast-time sampling rate
radar.Fslow = 1 / radar.T_chirp;               % slow-time sampling rate
radar.T_ch_eff = radar.N_samp / radar.Fs;
radar.S_slope = radar.S_slope_in_mhzus * 1e12; % S (Hz/s)
radar.BW = radar.S_slope * radar.T_ch_eff; % sweep bandwidth (Hz)

% FFT sizes (you can change/zero-pad)
radar.Nfft_range   = radar.N_samp;
radar.Nfft_doppler = radar.N_chirps;

% Radar Info
radar.R_res   = c / (2 * radar.BW);                % range resolution [m]
radar.R_max   = c * radar.Fs / (2 * radar.S_slope); % unambiguous range [m]

T_cpi         = radar.N_chirps * radar.T_chirp;
radar.v_res   = radar.lambda / (2 * T_cpi);        % Doppler resolution [m/s]
radar.v_max   = radar.lambda / (4 * radar.T_chirp);% unambiguous v (±v_max)

fprintf('Radar: Fc = %.2f GHz, BW = %.1f MHz\n', radar.fc/1e9, radar.BW/1e6);
fprintf('  Range res = %.3f m, R_max = %.1f m\n', radar.R_res, radar.R_max);
fprintf('  Doppler res = %.3f m/s, v_max = ±%.2f m/s\n', ...
        radar.v_res, radar.v_max);

%% ------------- 2. UAV PARAMETERS --------------------------------------
uav.R0        = 40;          % initial range (m)
uav.v0        = 2;           % initial radial velocity (m/s)
uav.a         = 2.5;         % constant radial acceleration (m/s^2) -> Doppler smearing
uav.beta_deg  = 0;           % angle between blade plane & LOS (deg)
uav.beta      = deg2rad(uav.beta_deg);

% Body scattering
uav.N_body_scatter = 50;
uav.body_rcs       = 1.0;    % relative RCS weight of the body

% Rotors & blades (micro-Doppler)
uav.N_rotors         = 4;    % quadcopter
uav.N_blades_per_rot = 4;    % blades per rotor
uav.N_scat_per_blade = 5;    % point scatterers along each blade
uav.blade_length     = 0.1;  % blade length (m)
uav.uav_rot_Hz       = 50;   % rotor rotation frequency (Hz) ~ 2400 rpm

% If we want "pure acceleration smear" with no rotor micro-Doppler,
% set uav.blade_rcs = 0 and/or uav.N_rotors = 0.
uav.blade_rcs = 0.25;  % total RCS weight of all blade scatterers

% (Optional) small rotor-to-rotor speed variations to create MD "smearing"
uav.rotor_freq_jitter = 0.0;   % ±10% variation between rotors

% Noise
noise.SNR_dB = 20;   % approximate SNR of beat signal [dB]

%% ------------- 3. BUILD SLOW-TIME GEOMETRY ----------------------------
numChirps = radar.N_chirps;
numSamples = radar.N_samp;

[RD_mag_dB, range_axis, ...
    vel_axis, doppler_freq_axis, aux] = simulate_uav_rd_mmwcas(radar, uav, noise);

%% ------------- 11. PLOTS ---------------------------------------------
% Range–Doppler map
figure;
surf(RD_mag_dB);
colormap("parula");
colorbar;
axis tight vis3d;
view(3);
rotate3d on;
% set(gca,'YDir','normal');
 

% % Zoom around the UAV range cell
% [~, idx_r0] = min(abs(range_axis - uav.R0));
% range_window = max(1, idx_r0-10):min(N_fft_range, idx_r0+10);
% 
% figure;
% imagesc(range_axis(range_window), vel_axis, RD_mag_dB(:, range_window));
% set(gca,'YDir','normal');
% xlabel('Range (m)');
% ylabel('Radial velocity (m/s)');
% title('Zoomed RD Map around UAV range bin');
% colormap spring; colorbar;

