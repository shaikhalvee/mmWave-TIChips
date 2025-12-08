%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mmWave UAV Accelerating Target Simulation (FMCW)
%
% - Models a UAV as a point target with constant acceleration along LOS.
% - Generates FMCW beat signal at baseband.
% - Adds noise (and optionally multipath & micro-Doppler).
% - Runs range FFT + Doppler FFT exactly like a typical mmWave pipeline.
% - Provides axes in meters and m/s for direct comparison to real data.
%
% You should:
%   1) Replace the PARAMS in Section 1 with your real radar settings.
%   2) Use the same windows / FFT sizes as in your real DSP chain.
%   3) Run the same processing on your measured data and compare.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; close all;

%% ========================= 1. RADAR PARAMETERS ==========================
c = 3e8;                % speed of light [m/s]

% ---- Replace these with your radar's actual parameters ----
fc = 77e9;         % carrier frequency [Hz]
lambda = c / fc;       % wavelength [m]

B = 1e9;          % chirp bandwidth [Hz]
T_chirp = 80e-6;        % chirp duration [s]
S = B / T_chirp;  % chirp slope [Hz/s]

fs = 2e6;          % ADC sampling rate [Hz]
Ns = round(T_chirp * fs);   % samples per chirp
Nchirps = 128;          % chirps per frame (slow-time length)

% FFT sizes (could use Ns, Nchirps, or zero-padded versions)
Nfft_range   = 2^nextpow2(Ns);
Nfft_doppler = 2^nextpow2(Nchirps);

% Range metrics (for info)
R_res   = c / (2 * B);                      % range resolution [m]
R_max   = c * fs / (2 * S);                 % max unambiguous range [m]
fprintf('Range resolution: %.3f m, max unambiguous range: %.1f m\n', ...
        R_res, R_max);

% Doppler metrics (for info)
T_CPI   = Nchirps * T_chirp;                % coherent processing interval
v_res   = lambda / (2 * T_CPI);             % Doppler resolution [m/s]
v_max   = lambda / (4 * T_chirp);           % unambiguous v (±v_max)
fprintf('Doppler resolution: %.3f m/s, max unambiguous velocity: ±%.2f m/s\n', ...
        v_res, v_max);

%% ===================== 2. TARGET / UAV MOTION MODEL =====================
% Simple 1D motion along radar line-of-sight:
% R(t) = R0 + v0 * t + 0.5 * a * t^2

R0   = 15;        % initial range [m]
v0   = 0.0;       % initial radial velocity [m/s]
a    = 3.0;       % radial acceleration [m/s^2] (adjust to match experiment)

% Optional: micro-Doppler (e.g., propeller) – small sinusoidal radial velocity
use_micro_doppler = true;
v_md_amp   = 0.3;      % amplitude of micro-Doppler [m/s]
f_md       = 80;       % micro-Doppler frequency [Hz] (related to blade rate)

% Optional: single multipath component (e.g., ground bounce)
use_multipath = true;
alpha_mp      = 0.3;   % relative amplitude of multipath
deltaR_mp     = 1.5;   % extra range for multipath [m]

% Target complex amplitude
A = 1.0;               % overall complex gain

%% ===================== 3. TIME GRIDS (FAST & SLOW) ======================
% We will store beat signal as a matrix: Ns x Nchirps
%  - rows: fast-time samples within chirp
%  - cols: slow-time index (chirp number)

n_fast = (0:Ns-1).';                  % fast-time sample indices (column)
m_slow = 0:(Nchirps-1);               % slow-time indices (row)
t_fast = n_fast / fs;                 % fast-time [s], Ns x 1
t_slow = m_slow * T_chirp;            % slow-time [s], 1 x Nchirps

% 2D time grid
t = t_fast + t_slow;                  % implicit expansion (Ns x Nchirps)

%% ===================== 4. TARGET RANGE / DELAY VS TIME ==================
% Base LOS motion
R = R0 + v0 .* t + 0.5 * a .* t.^2;   % Ns x Nchirps

% Add micro-Doppler as a time-varying extra radial velocity if enabled
if use_micro_doppler
    v_md = v_md_amp * sin(2*pi*f_md*t);   % Ns x Nchirps
    % Integrate micro-Doppler velocity into range approximately (small effect)
    % R_md ~ integral of v_md dt; numerically just adjust R via instantaneous v
    % For small v_md, simplest is to treat as extra delay term directly:
    R = R + (v_md ./ a) * 0;  % (placeholder: negligible contribution to range)
    % Instead, apply micro-Doppler as a phase term later (cleaner, see below).
end

tau = 2 * R / c;                  % round-trip delay [s], Ns x Nchirps

%% ================= 5. TRANSMIT / RECEIVE CHIRPS & BEAT SIGNAL ===========
% Ideal transmitted FMCW chirp (baseband, complex)
phi_tx = 2*pi * (fc * t + 0.5 * S .* t.^2);      % phase of tx signal
s_tx   = exp(1j * phi_tx);

% Received signal from main LOS scatterer (no multipath yet)
phi_rx = 2*pi * (fc * (t - tau) + 0.5 * S .* (t - tau).^2);
s_rx   = A * exp(1j * phi_rx);

% Optional micro-Doppler as a direct phase modulation:
if use_micro_doppler
    % micro-Doppler instantaneous phase: phi_md(t) = 2*pi * (2 * v_md / lambda) * t
    v_md = v_md_amp * sin(2*pi*f_md*t);
    f_md_inst = 2 * v_md / lambda;                % [Hz]
    phi_md = 2*pi * f_md_inst .* t;
    s_rx = s_rx .* exp(1j * phi_md);              % apply as phase modulation
end

% Beat signal after dechirp (mixer): tx * conj(rx)
s_beat_main = s_tx .* conj(s_rx);    % Ns x Nchirps

% Optional multipath: second echo with extra range deltaR_mp
if use_multipath
    R_mp   = R + deltaR_mp;
    tau_mp = 2 * R_mp / c;
    phi_rx_mp = 2*pi * (fc * (t - tau_mp) + 0.5 * S .* (t - tau_mp).^2);
    s_rx_mp   = alpha_mp * A * exp(1j * phi_rx_mp);
    if use_micro_doppler
        % Apply same micro-Doppler to multipath for simplicity
        s_rx_mp = s_rx_mp .* exp(1j * phi_md);
    end
    s_beat_mp = s_tx .* conj(s_rx_mp);
else
    s_beat_mp = zeros(size(s_beat_main));
end

% Total beat signal (complex baseband)
s_beat = s_beat_main + s_beat_mp;

%% ========================== 6. ADD NOISE ================================
SNR_dB = 20;   % desired SNR in dB (approximate, depends on scaling)
Ps = mean(abs(s_beat(:)).^2);
SNR_lin = 10^(SNR_dB/10);
sigma2 = Ps / SNR_lin;

noise = sqrt(sigma2/2) * (randn(size(s_beat)) + 1j*randn(size(s_beat)));
x = s_beat + noise;   % this is your simulated ADC data (complex)

%% ================== 7. RANGE FFT (FAST-TIME) PROCESSING =================
% Use same window and FFT length as your real pipeline
win_fast = hann(Ns);              % range window
x_win = x .* win_fast;            % Ns x Nchirps

% Range FFT along rows (fast-time)
Sig_range = fft(x_win, Nfft_range, 1);       % Nfft_range x Nchirps

% Keep only positive frequencies (0 .. Nfft_range/2-1) for range
k_range = 0:(Nfft_range/2 - 1);
Sig_range_pos = Sig_range(k_range+1, :);     % (Nfft_range/2) x Nchirps

% Range axis [m]
fb = k_range * (fs / Nfft_range);           % beat frequencies [Hz]
R_axis = c .* fb / (2 * S);                 % [m]

%% ================= 8. DOPPLER FFT (SLOW-TIME) PROCESSING =================
% Apply Doppler window across columns (chirps)
win_slow = hann(Nchirps).';                 % 1 x Nchirps
Sig_range_win = Sig_range_pos .* win_slow;  % implicit expansion

% Doppler FFT along columns (slow-time)
Sig_RD = fftshift(fft(Sig_range_win, Nfft_doppler, 2), 2); 
% Sig_RD: (Nfft_range/2) x Nfft_doppler

% Doppler frequency axis [Hz] & velocity axis [m/s]
fd = (-Nfft_doppler/2 : Nfft_doppler/2 - 1) * (1 / (Nfft_doppler * T_chirp));
v_axis = (lambda * fd) / 2;

%% ================== 9. VISUALIZATION: RANGE-DOPPLER MAP =================
% Choose a dynamic range for plotting (dB)
RD_dB = 20*log10(abs(Sig_RD) + eps);
maxVal = max(RD_dB(:));
dynRange = 40;   % dB

figure;
imagesc(v_axis, R_axis, RD_dB);
axis xy;
caxis([maxVal - dynRange, maxVal]);
xlabel('Velocity [m/s]');
ylabel('Range [m]');
title('Simulated Range-Doppler Map (UAV with Acceleration)');
colorbar;

%% =========== 10. 1D SLICES: MEASURE DOPPLER / RANGE SMEARING ============
% Pick the range bin where the UAV is strongest (in simulation we know it's near R0)
[~, idxR_peak] = min(abs(R_axis - R0));     % nearest range bin to R0
doppler_slice = abs(Sig_RD(idxR_peak, :)).^2;  % power vs Doppler

% Normalize and compute -3 dB width of Doppler slice
doppler_slice = doppler_slice / max(doppler_slice);
thresh = 10^(-3/10);  % -3 dB in linear
idx_above = find(doppler_slice >= thresh);

if numel(idx_above) >= 2
    v_width_3dB = v_axis(idx_above(end)) - v_axis(idx_above(1));
else
    v_width_3dB = 0;
end

fprintf('Approx. -3 dB Doppler width at R ~ %.2f m: %.4f m/s\n', ...
        R_axis(idxR_peak), v_width_3dB);

figure;
plot(v_axis, 10*log10(doppler_slice + eps), 'LineWidth', 1.5);
grid on;
xlabel('Velocity [m/s]');
ylabel('Magnitude [dB]');
title(sprintf('Doppler Slice at R \\approx %.2f m (Width_{-3dB} = %.3f m/s)', ...
      R_axis(idxR_peak), v_width_3dB));

% Similarly, you can look at a range slice near peak Doppler:
[~, idxV_peak] = max(doppler_slice);
range_slice = abs(Sig_RD(:, idxV_peak)).^2;
range_slice = range_slice / max(range_slice);
idx_above_R = find(range_slice >= thresh);

if numel(idx_above_R) >= 2
    R_width_3dB = R_axis(idx_above_R(end)) - R_axis(idx_above_R(1));
else
    R_width_3dB = 0;
end

figure;
plot(R_axis, 10*log10(range_slice + eps), 'LineWidth', 1.5);
grid on;
xlabel('Range [m]');
ylabel('Magnitude [dB]');
title(sprintf('Range Slice at v \\approx %.3f m/s (Width_{-3dB} = %.3f m)', ...
      v_axis(idxV_peak), R_width_3dB));

fprintf('Approx. -3 dB Range width at v ~ %.3f m/s: %.4f m\n', ...
        v_axis(idxV_peak), R_width_3dB);

%% ================= 11. ANALYTICAL SMEARING (FOR COMPARISON) =============
% Rough theory for acceleration-induced smear (no windowing, single point):
%
% Total range change over CPI:
%   dR_total = v0*T_CPI + 0.5*a*T_CPI^2
% Range bins spanned:
%   N_R_theory = dR_total / R_res
%
% Total Doppler change over CPI:
%   dv_total = a * T_CPI
% Velocity bins spanned:
%   N_D_theory = dv_total / v_res
%
% This is a crude estimate but useful to sanity-check your measurements.

dR_total = v0 * T_CPI + 0.5 * a * T_CPI^2;
N_R_theory = dR_total / R_res;

dv_total = a * T_CPI;
N_D_theory = dv_total / v_res;

fprintf('\nTHEORETICAL ESTIMATES (no windowing, ideal processing):\n');
fprintf(' Total range change: %.3f m -> spans ~%.2f range bins\n', ...
        dR_total, N_R_theory);
fprintf(' Total velocity change: %.3f m/s -> spans ~%.2f Doppler bins\n', ...
        dv_total, N_D_theory);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of simulation script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
