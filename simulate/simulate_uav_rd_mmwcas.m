function [RD_mag_dB, range_axis, vel_axis, doppler_freq_axis, aux] = simulate_uav_rd_mmwcas(radar, uav, noise)
%% ------------- 3. BUILD SLOW-TIME GEOMETRY ----------------------------
numChirps = radar.N_chirps;
numSamples = radar.N_samp;
c = physconst('LightSpeed');

% We approximate micro-motion and body motion as constant within each chirp,
% and sample them at the *chirp center* times (better for Doppler).
t_slow = (0:numChirps-1) * radar.T_chirp;  % slow-time (per chirp) start from 0. it's time, not an array
t_center = t_slow + 0.5 * radar.T_chirp;

% Body range vs slow-time with constant acceleration
R_body = uav.R0 + uav.v0 .* t_center + 0.5 * uav.a .* t_center.^2; % 1 x numChirps

%% ------------- 4. BUILD SCATTERER SET --------------------------------
% We represent the UAV as:
%   - N_body_scatter point scatterers at R_body(k)
%   - Rotating blades: rotors × blades × point scatterers along each blade
%
% the micro-motion term for each blade scatterer is
%   R_MD(t) = - r * cos(beta) * cos(omega * t + phi0) (eq. analogous to (4)/(6)).

% assert that there's no 0 for body & rotor.
assert(uav.N_rotors ~= 0, 'number of rotors cannot be 0');
assert(uav.N_blades_per_rot ~= 0, 'number of blades cannot be 0');
assert(uav.N_body_scatter ~= 0, 'number of body scatterer cannot be 0');
assert(uav.N_scat_per_blade ~= 0, 'number of rotor scatterer cannot be 0');

% 4.1 Body scatterers (no micro-motion)
N_body = uav.N_body_scatter;
A_body_each = uav.body_rcs / N_body;    % rcs ampl signal for each scatterer

% 4.2 Blade scatterers
N_rotor = uav.N_rotors;
N_blades = uav.N_blades_per_rot;
N_scatterer_per_blade = uav.N_scat_per_blade; % Scatterer index along blade

% total rotor scatterers
N_rotor_scatterer = N_rotor * N_blades * N_scatterer_per_blade;
% in total scatterers
N_total_scatterer = N_body + N_rotor_scatterer;


% Allocate arrays for rotor scatterer parameters
% vector of each scatterer to the radar -> (radius, rotor angle, angular vel of each scatterer)
r_list = zeros(N_rotor_scatterer, 1);  % radius along blade (r_p,b,q​)
phi0_list = zeros(N_rotor_scatterer, 1);  % initial rotor angle (𝜙_p,b,q)​
omega_list = zeros(N_rotor_scatterer, 1);  % angular velocities (rad/s) (ω_p)


%%% for each rotor->for each blade->for each scatter per blade :->
% Allocate geometry for rotor scatterers (Rotor frequency with jitter)
% need to change later for the smearing. becoz each rotor should have
% different angular frequencies according to the rotor freq & accel
idx = 1;    % global index for scatterer
for p = 1:N_rotor
    %% phase calculation
    % Per-rotor angular frequency with jitter
    f_rot_p = uav.uav_rot_Hz * (1 + uav.rotor_freq_jitter * (2*rand-1)); % p'th rotor frequency
    omega_p = 2*pi*f_rot_p;     % rotor omega for the freq above
    rotor_phase_offset = 2*pi*rand; % random overall phase per rotor (pth rotor)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This models what Kang calls rotors "with different rotational speeds",
    % which leads to spectral smearing because each rotor's micro-Doppler lines
    % are at slightly different chopping frequencies. Kang explicitly points
    % out that when multiple rotors have different $(ω_p)$, the high-frequency
    % part of the spectrum smears and loses clean comb structure.
    % (That's what the jitter is trying to emulate.)
    % So this part is consistent with Kang's observation but the exact
    % distribution (uniform jitter, chosen by us) is not directly from a
    % formula in Kang—it's a reasonable modeling choice.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for blade_q = 1:N_blades
        blade_phase0 = rotor_phase_offset + 2*pi*(blade_q-1)/N_blades;  % evenly spaced blades, that's why the number is (b-1/N * 2pi)

        % Radial Positions: scatterers along the blade (avoid exact center = 0)
        r_vals = linspace(0.2*uav.blade_length, uav.blade_length, N_scatterer_per_blade);

        for rr = 1:N_scatterer_per_blade
            r_list(idx) = r_vals(rr);   % scatterer radius list
            phi0_list(idx) = blade_phase0;  % initial rotor phase (𝜙_p,b,q)
            omega_list(idx) = omega_p;
            idx = idx + 1;
        end
    end
end


% Assemble amplitude vector for all scatterers
A_blade_each = uav.blade_rcs / N_rotor_scatterer;
A_list = [A_body_each * ones(N_body,1) ; (A_blade_each) * ones(N_rotor_scatterer,1)];


%% ------------- 5. MICRO-DOPPLER DISPLACEMENTS ------------------------
% For body scatterers: no micro-motion, so MD displacement = 0
R_md = zeros(N_total_scatterer, numChirps);   % (scatterer x num chirps/doppFFT)

% Rotor scatterers: apply micro-motion model
beta = uav.beta;    % UAV angle along the LOS to the radar
for s = 1:N_rotor_scatterer     % build the UAV signal for rotor
    s_idx = N_body + s;  % index in global scatterer list. They are [N_body; N_rotor], so placed index is (N_body + s)
    r_s = r_list(s);
    omega_s = omega_list(s);
    phi0_s  = phi0_list(s);     % rotor initial angle (this is not the phase, phase will be calculated later)

    % micro-motion displacement vs slow-time
    % R_MD(t) = - r * cos(beta) * cos(omega t + phi0)
    R_md(s_idx, :) = - r_s * cos(beta) * cos(omega_s * t_slow + phi0_s);
end

%% ------------- 6. SCATTERER PHASE VS SLOW-TIME -----------------------
lambda = radar.lambda;

% Expand body range to (scatterer x slow-time) through broadcasting
R_body_mat = repmat(R_body, N_total_scatterer, 1);  % S x numChirps

% Total range per scatterer = body + micro-motion
R_total = R_body_mat + R_md;   % S x numChirps

% Phase (due to 4*pi*R/lambda) as in canonical micro-Doppler model
phase_scat = 4*pi * R_total / lambda;   % radians

% Complex sum over scatterers to get effective per-chirp "target phasor"
% A_eff(k) = sum_s A_s * exp(j * phase_scat(s,k))
A_eff = (A_list.' * exp(1j * phase_scat));   % 1 x numChirps

%% ------------- 7. FAST-TIME (RANGE) BEAT FREQUENCY -------------------
% Use classic FMCW beat frequency model:
%   f_b = 2 * S * R0 / c
% And assume same beat frequency for all scatterers (range differences are tiny).

f_b_Hz   = 2 * radar.S_slope * uav.R0 / c;   % beat frequency for R0
f_b_norm = f_b_Hz / radar.Fs;                % cycles/sample

n = 0:numSamples-1;                          % ADC sample number for indexing
fast_phase = 2*pi * f_b_norm * n;            % 1 x numSamples

% Full beat signal matrix: [chirp x sample]
% s(k,n) = A_eff(k) * exp(j * 2*pi*f_b * n/Fs)
S = (A_eff.' * exp(1j * fast_phase));        % numChirps x numSamples
% S = S.';        % numSamples x numChirps

%% ------------- 8. ADD NOISE ------------------------------------------
signal_power = mean(abs(S(:)).^2);
sigma2 = signal_power / (10^(noise.SNR_dB/10));
noise_cplx = sqrt(sigma2/2) * (randn(size(S)) + 1j*randn(size(S)));

S_noise = S + noise_cplx;

%% ------------- 9. RANGE–DOPPLER PROCESSING ---------------------------
% Apply windowing in both dimensions
win_range   = hann(numSamples).';      % 1 x numSamples
win_doppler = hann(numChirps);       % numChirps x 1

% S_win -> simulated IF signal
S_win = (win_doppler * win_range) .* S_noise;
S_win = S_win.';    % switching to N_fft_range x N_fft_dopp [standard size]

% FFT sizes
N_fft_range  = radar.Nfft_range;
N_fft_dopp   = radar.Nfft_doppler;

% Range FFT (across fast-time)
S_range = fft(S_win, N_fft_range, 1);           % N_fft_range x numChirps

% Doppler FFT (across chirps)
RD_cube = fftshift(fft(S_range, N_fft_dopp, 2), 2);  % N_fft_range x N_fft_dopp

RD_mag_dB = 20*log10(abs(RD_cube) + 1e-12);
% RD_mag_dB = RD_mag_dB.';

%% ------------- 10. AXES ----------------------------------------------
% Range axis
range_res = c / (2 * radar.BW);
range_axis = (0:N_fft_range-1) * range_res;      % (m)

% Doppler / velocity axis
doppler_freq_axis = (-N_fft_dopp/2 : N_fft_dopp/2-1) * (radar.Fslow / N_fft_dopp);  % Hz
vel_axis = doppler_freq_axis * radar.lambda / 2;      % m/s

% Pack auxiliary info if needed
aux.R_body          = R_body;
aux.A_eff           = A_eff;
aux.R_total         = R_total;
aux.phase_scat      = phase_scat;
aux.noise_variance  = sigma2;
aux.S               = S;
aux.S_noise         = S_noise;
aux.S_range         = S_range;
aux.RD_cube         = RD_cube;
end