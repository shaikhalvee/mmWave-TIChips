%% UAV Range–Doppler Simulator (FMCW, micro-Doppler + Doppler smearing)
% This script:
%   1) defines radar & UAV parameters,
%   2) simulates the complex FMCW beat signal,
%   3) computes the range–Doppler map,
%   4) plots the RD heatmap.

clear; clc; close all;

%% ------------- 1. RADAR PARAMETERS ------------------------------------
c = physconst('LightSpeed');            % speed of light (m/s)

radar.fc       = 77e9;      % carrier (Hz)
radar.lambda   = c / radar.fc;
radar.N_samp = 512;         % ADC samples per chirp (fast-time)
radar.Fs = 16e3;            % fast-time sampling rate or ADC sampling rate
radar.S_slope_in_mhzus = 7; % Chirp slope S (MHz/us)
radar.T_chirp  = 250e-6;     % chirp duration (s)  (short to capture rotor MD)
radar.N_chirps = 256;       % number of chirps in CPI (slow-time)

% Derived quantities
% radar.S_slope  = radar.BW / radar.T_chirp;        % chirp slope S (Hz/s)
% radar.Fs       = radar.N_samp / radar.T_chirp;    % fast-time sampling rate
radar.Fslow    = 1 / radar.T_chirp;               % slow-time sampling rate
radar.T_ch_eff = radar.N_samp / radar.Fs;
radar.S_slope = radar.S_slope_in_mhzus * 1e12; % S (Hz/s)
radar.BW = radar.S_slope / radar.T_ch_eff; % sweep bandwidth (Hz)

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
uav.N_blades_per_rot = 2;    % blades per rotor
uav.N_scat_per_blade = 6;    % point scatterers along each blade
uav.blade_length     = 0.12; % blade length (m)
uav.f_rot_Hz         = 40;   % rotor rotation frequency (Hz) ~ 2400 rpm
uav.blade_rcs        = 0.2;  % total RCS weight of all blade scatterers

% (Optional) small rotor-to-rotor speed variations to create MD "smearing"
uav.rotor_freq_jitter = 0.05;   % ±5% variation between rotors

% Noise
noise.SNR_dB         = 20;   % SNR of received beat signal (dB)

%% ------------- 3. BUILD SLOW-TIME GEOMETRY ----------------------------
Nc = radar.N_chirps;
Ns = radar.N_samp;

t_slow = (0:Nc-1) * radar.T_chirp;  % slow-time (per chirp)

% Body range vs slow-time with constant acceleration
R_body = uav.R0 + uav.v0 .* t_slow + 0.5 * uav.a .* t_slow.^2;

%% ------------- 4. BUILD SCATTERER SET --------------------------------
% We represent the UAV as:
%   - N_body_scatter point scatterers at R_body(k)
%   - Rotating blades: rotors × blades × point scatterers along each blade
%
% Following Kang & He, the micro-motion term for each blade scatterer is
%   R_MD(t) = - r * cos(beta) * cos(omega * t + phi0)    (eq. analogous to (4)/(6)).

% 4.1 Body scatterers (no micro-motion)
N_body = uav.N_body_scatter;
A_body_each = uav.body_rcs / N_body;

% 4.2 Blade scatterers
N_rot   = uav.N_rotors;
N_blade = uav.N_blades_per_rot;
N_rad   = uav.N_scat_per_blade;

% total rotor scatterers
N_rotor_scat = N_rot * N_blade * N_rad;

% Allocate arrays for rotor scatterer parameters
r_list    = zeros(N_rotor_scat,1);  % radius along blade
phi0_list = zeros(N_rotor_scat,1);  % initial rotor angle
omega_list= zeros(N_rotor_scat,1);  % angular velocities (rad/s)

idx = 1;
for p = 1:N_rot
    % Rotor-specific angular frequency with jitter
    f_rot_p = uav.f_rot_Hz * (1 + uav.rotor_freq_jitter * (2*rand-1));
    omega_p = 2*pi*f_rot_p;
    rotor_phase_offset = 2*pi*rand; % random overall phase per rotor

    for b = 1:N_blade
        blade_phase0 = rotor_phase_offset + 2*pi*(b-1)/N_blade;  % evenly spaced blades

        % scatterers along the blade (avoid exact center = 0)
        r_vals = linspace(0.2*uav.blade_length, uav.blade_length, N_rad);

        for rr = 1:N_rad
            r_list(idx)     = r_vals(rr);
            phi0_list(idx)  = blade_phase0;
            omega_list(idx) = omega_p;
            idx = idx + 1;
        end
    end
end

N_scat_total = N_body + N_rotor_scat;

% Assemble amplitude vector for all scatterers
A_list = [ A_body_each * ones(N_body,1) ; ...
           (uav.blade_rcs / N_rotor_scat) * ones(N_rotor_scat,1) ];

%% ------------- 5. MICRO-DOPPLER DISPLACEMENTS ------------------------
% For body scatterers: no micro-motion, so MD displacement = 0
R_md = zeros(N_scat_total, Nc);   % (scatterer x slow-time)

% Rotor scatterers: apply micro-motion model
beta = uav.beta;
for s = 1:N_rotor_scat
    s_idx = N_body + s;  % index in global scatterer list
    r_s   = r_list(s);
    omega_s = omega_list(s);
    phi0_s  = phi0_list(s);

    % micro-motion displacement vs slow-time
    % R_MD(t) = - r * cos(beta) * cos(omega t + phi0)
    R_md(s_idx, :) = - r_s * cos(beta) * cos(omega_s * t_slow + phi0_s);
end

%% ------------- 6. SCATTERER PHASE VS SLOW-TIME -----------------------
lambda = radar.lambda;

% Expand body range to (scatterer x slow-time) through broadcasting
R_body_mat = repmat(R_body, N_scat_total, 1);  % S x Nc

% Total range per scatterer = body + micro-motion
R_total = R_body_mat + R_md;   % S x Nc

% Phase (due to 4*pi*R/lambda) as in canonical micro-Doppler model
phase_scat = 4*pi * R_total / lambda;   % radians

% Complex sum over scatterers to get effective per-chirp "target phasor"
% A_eff(k) = sum_s A_s * exp(j * phase_scat(s,k))
A_eff = (A_list.' * exp(1j * phase_scat));   % 1 x Nc

%% ------------- 7. FAST-TIME (RANGE) BEAT FREQUENCY -------------------
% Use classic FMCW beat frequency model:
%   f_b = 2 * S * R0 / c
% And assume same beat frequency for all scatterers (range differences are tiny).

f_b_Hz   = 2 * radar.S_slope * uav.R0 / c;   % beat frequency for R0
f_b_norm = f_b_Hz / radar.Fs;                % cycles/sample

n = 0:Ns-1;
fast_phase = 2*pi * f_b_norm * n;            % 1 x Ns

% Full beat signal matrix: [chirp x sample]
% s(k,n) = A_eff(k) * exp(j * 2*pi*f_b * n/Fs)
S = (A_eff.' * exp(1j * fast_phase));        % Nc x Ns

%% ------------- 8. ADD NOISE ------------------------------------------
signal_power = mean(abs(S(:)).^2);
sigma2 = signal_power / (10^(noise.SNR_dB/10));
noise_cplx = sqrt(sigma2/2) * (randn(size(S)) + 1j*randn(size(S)));

S_noisy = S + noise_cplx;

%% ------------- 9. RANGE–DOPPLER PROCESSING ---------------------------
% Apply windowing in both dimensions
win_range   = hann(Ns).';      % 1 x Ns
win_doppler = hann(Nc);        % Nc x 1

S_win = (win_doppler * win_range) .* S_noisy;

% FFT sizes
N_fft_range  = 512;
N_fft_dopp   = 512;

% Range FFT (across fast-time)
S_range = fft(S_win, N_fft_range, 2);           % Nc x N_fft_range

% Doppler FFT (across chirps)
RD_cube = fftshift(fft(S_range, N_fft_dopp, 1), 1);  % N_fft_dopp x N_fft_range

RD_mag_dB = 20*log10(abs(RD_cube) + 1e-12);

%% ------------- 10. AXES ----------------------------------------------
% Range axis
range_res = c / (2 * radar.BW);
range_axis = (0:N_fft_range-1) * range_res;      % (m)

% Doppler / velocity axis
doppler_axis = (-N_fft_dopp/2 : N_fft_dopp/2-1) * (radar.Fslow / N_fft_dopp);  % Hz
vel_axis = doppler_axis * radar.lambda / 2;      % m/s

%% ------------- 11. PLOTS ---------------------------------------------
% Range–Doppler map
figure;
imagesc(range_axis, vel_axis, RD_mag_dB);
set(gca,'YDir','normal');
xlabel('Range (m)');
ylabel('Radial velocity (m/s)');
title('Simulated UAV Range–Doppler Map');
colormap jet; colorbar;

% Zoom around the UAV range cell
[~, idx_r0] = min(abs(range_axis - uav.R0));
range_window = max(1, idx_r0-10):min(N_fft_range, idx_r0+10);

figure;
imagesc(range_axis(range_window), vel_axis, RD_mag_dB(:, range_window));
set(gca,'YDir','normal');
xlabel('Range (m)');
ylabel('Radial velocity (m/s)');
title('Zoomed RD Map around UAV range bin');
colormap jet; colorbar;

