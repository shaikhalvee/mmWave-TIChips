%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   UAV Range–Doppler Simulator for MMWCAS-RF-EVM (Doppler-focused)
%
%   - Radar model: FMCW (He-style sensing model, TI MMWCAS-RF-EVM-like)
%   - UAV model  : Kang-style geometry for rotor micro-motion
%   - Range is intentionally kept (approximately) fixed so you only see
%                  Doppler smearing / micro-Doppler, not range migration.
%   - Output     : Range–Doppler heatmap, plus axes and cubes for analysis.
%
%   Interpret this as one virtual RX channel / beam from the 9-TX ULA.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; close all;

c0 = 3e8;        % speed of light [m/s]

%% ===================== 1. RADAR PARAMETERS (MMWCAS-like) ===============
% These defaults roughly follow typical MMWCAS-RF-EVM configs:
%   fc ~ 77 GHz, slope ~ 15 MHz/us, 256 ADC samples per chirp, Fs ~ 6 MHz,
%   giving ~0.3 m range resolution and ~few m/s Doppler span. See e.g.
%   azinke/mmwave default config for MMWCAS-DSP/RF-EVM. 

radar.fc      = 77e9;          % carrier [Hz]
radar.lambda  = c0 / radar.fc; % wavelength [m]

radar.slope   = 15.0148e12;    % chirp slope S [Hz/s] (15.0148 MHz/us)
radar.fs      = 6e6;           % ADC sampling rate [Hz]

radar.Ns      = 256;           % ADC samples per chirp (fast-time)
radar.Nc      = 128;           % chirps per frame (slow-time / CPI)

radar.T_chirp = radar.Ns / radar.fs;    % chirp duration [s] (approx ramp time)
radar.B       = radar.slope * radar.T_chirp;  % sweep bandwidth [Hz]

% FFT sizes (you can change/zero-pad)
radar.Nfft_range   = 512;
radar.Nfft_doppler = 512;

% Info (for sanity)
radar.R_res   = c0 / (2 * radar.B);                % range resolution [m]
radar.R_max   = c0 * radar.fs / (2 * radar.slope); % unambiguous range [m]

T_CPI         = radar.Nc * radar.T_chirp;
radar.v_res   = radar.lambda / (2 * T_CPI);        % Doppler resolution [m/s]
radar.v_max   = radar.lambda / (4 * radar.T_chirp);% unambiguous v (±v_max)

fprintf('Radar: fc = %.2f GHz, B = %.1f MHz\n', radar.fc/1e9, radar.B/1e6);
fprintf('  Range res = %.3f m, R_max = %.1f m\n', radar.R_res, radar.R_max);
fprintf('  Doppler res = %.3f m/s, v_max = ±%.2f m/s\n', ...
        radar.v_res, radar.v_max);

%% ================= 2. UAV (Kang-like) PARAMETERS =======================
% Motion is along radar line of sight (LOS).
% You can plug in your own R0, v0, a from your measurements / estimates.

uav.R0        = 40;           % initial range [m]
uav.v0        = 2.0;          % initial radial velocity [m/s]
uav.a         = 0.8;          % radial acceleration [m/s^2] (Doppler smear)

% Aspect: angle between rotor tip-path plane and LOS (Kang's beta).
uav.beta_deg  = 15;           % [deg]
uav.beta      = deg2rad(uav.beta_deg);

% Body scattering (several points on fuselage)
uav.N_body_scatter = 5;
uav.body_rcs        = 1.0;    % total body RCS weight (relative)

% Rotors / blades (micro-Doppler)
uav.N_rotors          = 4;     % e.g., quadcopter
uav.N_blades_per_rot  = 2;     % blades per rotor
uav.N_scat_per_blade  = 6;     % scatterers along each blade
uav.blade_length      = 0.12;  % [m]
uav.f_rot_mean_Hz     = 40;    % mean rotor frequency [Hz] (~2400 rpm)
uav.rotor_freq_jitter = 0.05;  % ±5% per-rotor variation

uav.blade_rcs         = 0.25;  % total rotor RCS weight

% If you want "pure acceleration smear" with no rotor micro-Doppler,
% set uav.blade_rcs = 0 and/or uav.N_rotors = 0.

%% =================== 3. NOISE PARAMETERS ===============================
noise.SNR_dB = 20;            % approximate SNR of beat signal [dB]

%% =================== 4. RUN SIMULATION ================================
[RD_dB, range_axis, vel_axis, Sig_RD, S_noisy, aux] = ...
    simulate_uav_rd_mmwcas(radar, uav, noise);

%% =================== 5. PLOT RANGE–DOPPLER MAPS =======================

% Full RD map
figure;
imagesc(range_axis, vel_axis, RD_dB);
set(gca,'YDir','normal');
xlabel('Range [m]');
ylabel('Radial velocity [m/s]');
title('Simulated UAV Range–Doppler (MMWCAS-RF-EVM-like)');
colorbar;

% Zoom around the UAV's range cell (where we *lock* the target)
[~, idxR0] = min(abs(range_axis - uav.R0));
zoom_win = max(1, idxR0-10) : min(numel(range_axis), idxR0+10);

figure;
imagesc(range_axis(zoom_win), vel_axis, RD_dB(:, zoom_win));
set(gca,'YDir','normal');
xlabel('Range [m]');
ylabel('Radial velocity [m/s]');
title('Zoomed RD around UAV range bin (Doppler-focused)');
colorbar;

%% =================== 6. 1D DOPPLER SLICE & SMEAR ======================

% Take Doppler slice at the locked range bin (idxR0)
doppler_slice_lin = abs(Sig_RD(:, idxR0)).^2;
doppler_slice_lin = doppler_slice_lin / max(doppler_slice_lin);

thresh_lin = 10^(-3/10);  % -3 dB threshold
idx_above  = find(doppler_slice_lin >= thresh_lin);

if numel(idx_above) >= 2
    v_width_3dB = vel_axis(idx_above(end)) - vel_axis(idx_above(1));
else
    v_width_3dB = 0;
end

figure;
plot(vel_axis, 10*log10(doppler_slice_lin + eps), 'LineWidth', 1.5);
grid on;
xlabel('Radial velocity [m/s]');
ylabel('Magnitude [dB]');
title(sprintf('Doppler slice at R \\approx %.2f m (Width_{-3 dB} = %.3f m/s)', ...
      range_axis(idxR0), v_width_3dB));

fprintf('\nApprox. -3 dB Doppler width at R ~ %.2f m: %.4f m/s\n', ...
        range_axis(idxR0), v_width_3dB);

%% =================== 7. OPTIONAL: THEORETICAL SMEAR ===================
% Very rough acceleration-only smear estimate (ignoring micro-Doppler):
%   total velocity change : dv = a * T_CPI
%   velocity resolution   : dv_res = lambda / (2*T_CPI)
%   bins spanned          : N_D_theory = dv / dv_res

dv_total   = uav.a * (radar.Nc * radar.T_chirp);
N_D_theory = dv_total / radar.v_res;

fprintf('Theoretical bins spanned by acceleration alone (no MD): %.2f\n', ...
        N_D_theory);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                    FUNCTION DEFINITIONS BELOW                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [RD_dB, range_axis, vel_axis, Sig_RD, S_noisy, aux] = ...
    simulate_uav_rd_mmwcas(radar, uav, noise)
%SIMULATE_UAV_RD_MMWCAS
%   Combines:
%     - He-style FMCW sensing (but with range locked to one bin)
%     - Kang-style micro-motion geometry for UAV rotors
%   and generates a Range–Doppler map.
%
%   Range smear is intentionally suppressed by:
%     - using a constant beat frequency for the body range R0
%     - encoding all motion (body + micro) into the *slow-time* phase
%       via exp(j*4*pi*R(t)/lambda).
%
%   This way, you see Doppler smearing and micro-Doppler around a
%   single range bin — exactly what you want for Doppler-focused analysis.

    c0 = 3e8;
    lam = radar.lambda;

    Ns = radar.Ns;
    Nc = radar.Nc;
    T_chirp = radar.T_chirp;
    fs = radar.fs;

    % --------- Slow-time grid (chirp index -> time) ----------
    % We approximate micro-motion and body motion as constant within each chirp,
    % and sample them at the *chirp center* times (better for Doppler).
    t_slow = (0:Nc-1) * T_chirp;          % [s], chirp start
    t_center = t_slow + 0.5*T_chirp;      % [s], chirp center

    % --------- Body motion along LOS -------------------------
    % R_body(t) = R0 + v0 t + 0.5 a t^2
    R_body = uav.R0 + uav.v0 * t_center + 0.5 * uav.a * t_center.^2; % 1 x Nc

    % --------- Scatterer set (Kang-like model) --------------
    % 1) Body scatterers: at R_body(t), no micro-motion
    N_body = uav.N_body_scatter;
    A_body_each = uav.body_rcs / max(N_body,1);

    % 2) Rotor scatterers: rotors × blades × radial points
    N_rot   = uav.N_rotors;
    N_blade = uav.N_blades_per_rot;
    N_rad   = uav.N_scat_per_blade;

    if N_rot == 0 || uav.blade_rcs == 0
        N_rotor_scat = 0;
    else
        N_rotor_scat = N_rot * N_blade * N_rad;
    end

    N_scat_total = N_body + N_rotor_scat;

    % Allocate geometry for rotor scatterers
    r_list     = zeros(N_rotor_scat,1);   % radius along blade
    phi0_list  = zeros(N_rotor_scat,1);   % initial rotor angle
    omega_list = zeros(N_rotor_scat,1);   % angular velocity [rad/s]

    idx = 1;
    for p = 1:N_rot
        % Per-rotor angular freq with small jitter
        f_rot_p = uav.f_rot_mean_Hz * (1 + uav.rotor_freq_jitter * (2*rand-1));
        omega_p = 2*pi*f_rot_p;
        rotor_phase_offset = 2*pi*rand;  % random rotor phase

        for b = 1:N_blade
            blade_phase0 = rotor_phase_offset + 2*pi*(b-1)/N_blade;

            % radial positions along the blade (avoid exact center)
            r_vals = linspace(0.2*uav.blade_length, uav.blade_length, N_rad);

            for rr = 1:N_rad
                r_list(idx)     = r_vals(rr);
                phi0_list(idx)  = blade_phase0;
                omega_list(idx) = omega_p;
                idx = idx + 1;
            end
        end
    end

    % Amplitudes for all scatterers (body + blades)
    if N_scat_total > 0
        A_list = [ A_body_each * ones(N_body,1) ; ...
                   (uav.blade_rcs / max(N_rotor_scat,1)) * ones(N_rotor_scat,1) ];
    else
        A_list = [];
    end

    % --------- Micro-motion displacements (per Kang) --------
    % R_MD(t) = - r * cos(beta) * cos(omega t + phi0)
    beta = uav.beta;

    R_md = zeros(N_scat_total, Nc);   % each row: one scatterer, col: chirp

    % Body scatterers have no micro-motion: R_md(1:N_body,:) = 0

    for s = 1:N_rotor_scat
        s_idx = N_body + s;   % global index of rotor scatterer
        r_s   = r_list(s);
        omega_s = omega_list(s);
        phi0_s  = phi0_list(s);

        R_md(s_idx, :) = - r_s * cos(beta) .* cos(omega_s * t_center + phi0_s);
    end

    % --------- Total range per scatterer vs slow-time -------
    % We approximate rotor centers as co-located with UAV body (offset only
    % gives static phase). So:
    %   R_total(s, k) = R_body(k) + R_md(s, k)

    R_body_mat = repmat(R_body, N_scat_total, 1);   % S x Nc
    R_total    = R_body_mat + R_md;                 % S x Nc

    % --------- Scatterer phase vs slow-time -----------------
    % Narrowband / Doppler phase:
    %   phi_s(k) = 4*pi * R_total(s,k) / lambda
    % This is the part that, across chirps, produces body Doppler + micro-Doppler.
    phase_scat = 4*pi * R_total / lam;             % S x Nc

    if N_scat_total > 0
        A_eff = (A_list.' * exp(1j * phase_scat)); % 1 x Nc, sum over scatterers
    else
        A_eff = zeros(1, Nc);
    end

    % Normalise so that |A_eff| is ~1 on average (just for SNR sanity)
    if any(A_eff ~= 0)
        A_eff = A_eff / mean(abs(A_eff));
    else
        A_eff = ones(1, Nc);
    end

    % --------- Fast-time beat signal (FMCW, but range-locked) ----------
    % He-style beat signal is:
    %   s_bf(t) = alpha * exp( j * (4*pi/c) * (f_c + S t) * R(t) )
    %
    % Here we *lock* the beat frequency to a constant range R0 so the target
    % stays in one range bin. All motion R(t) is encoded in A_eff(k) instead.
    %
    % Beat frequency for reference range R0:
    f_b = 2 * radar.slope * uav.R0 / c0;          % [Hz]
    n_fast = 0:Ns-1;
    fast_phase = 2*pi * (f_b / fs) * n_fast;      % 1 x Ns

    % Full beat signal: S(k,n) = A_eff(k) * exp(j*fast_phase(n))
    S = (A_eff.' * exp(1j * fast_phase));         % Nc x Ns

    % --------- Add complex Gaussian noise -------------------
    signal_power = mean(abs(S(:)).^2);
    if isfield(noise, 'SNR_dB')
        SNR_lin = 10^(noise.SNR_dB/10);
        sigma2  = signal_power / max(SNR_lin, eps);
    else
        sigma2  = 0;
    end
    noise_cplx = sqrt(sigma2/2) * (randn(size(S)) + 1j*randn(size(S)));

    S_noisy = S + noise_cplx;                     % Nc x Ns

    % --------- Range–Doppler Processing (He-style) ----------
    % 1) Range FFT along fast-time (columns)
    win_fast = hann(Ns).';                        % 1 x Ns
    win_slow = hann(Nc);                          % Nc x 1

    S_win = (win_slow * win_fast) .* S_noisy;     % Nc x Ns

    Nfft_r = radar.Nfft_range;
    Nfft_d = radar.Nfft_doppler;

    % Range FFT (fast-time)
    Sig_range = fft(S_win, Nfft_r, 2);            % Nc x Nfft_r

    % Doppler FFT (slow-time)
    Sig_RD = fftshift(fft(Sig_range, Nfft_d, 1), 1); % Nfft_d x Nfft_r

    % Magnitude in dB
    RD_mag = abs(Sig_RD);
    RD_dB  = 20*log10(RD_mag + 1e-12);

    % --------- Axes: range & velocity -----------------------
    B = radar.B;
    range_res = c0 / (2 * B);
    range_axis = (0:Nfft_r-1) * range_res;       % [m]

    fD = (-Nfft_d/2 : Nfft_d/2-1) * (1 / (Nfft_d * T_chirp));  % Doppler [Hz]
    vel_axis = (lam * fD) / 2;                   % [m/s]

    % Pack auxiliary info if needed
    aux.R_body          = R_body;
    aux.A_eff           = A_eff;
    aux.R_total         = R_total;
    aux.phase_scat      = phase_scat;
    aux.noise_variance  = sigma2;
    aux.S               = S;
    aux.S_noisy         = S_noisy;
    aux.Sig_range       = Sig_range;
end
