%% ——— Angle‐PMM Extraction (Capon + folding) —————————————————————
% Inputs assumed in workspace:
%   RDcube_cplx      % [R × D × F × Rx] complex Range–Doppler data
%   track_pf_range   % [1 × F] smoothed range in meters
%   Rres             % range resolution (m/bin)
%   mmWave_device    % has .lambda
% ------------------------------------------------------------

clear all;

%% Load Data
S = load('output/fft_result_cube.mat');
P = load('output/UAVtracking.mat');
mmWave_device = S.mmWave_device;
RDcube_cplx = S.fft_complex_radar_cube;
Rres = mmWave_device.range_res;
track_pf_range = P.track_pf_range;

%% PARAMETERS -------------------------------------------------------------
[R, D, F, Rx] = size(RDcube_cplx);  % radar cube size R x D x F x Rx
lambda = mmWave_device.lambda;      % wavelength (m)
d = lambda/2;                       % element spacing (m)  (ISK‑ODS)
azGrid = -90:1:90;                  % scan grid (deg)
La = numel(azGrid);                 % # angle bins La = numAngleBins
jMin = 2;   
jMax = 20;                          % PMM fold search (same as before)
snapWin = 8;                        % Doppler snapshots for R̂
delta = 1e-3;                       % diagonal loading
% deltaFrac = 1e-2; svdTol = 1e-3;    % for robust inversion
% Td = (mmWave_device.chirp_ramp_time + mmWave_device.chirp_idle_time) *1e-6 * mmWave_device.num_chirp_per_frame;
Td = mmWave_device.frame_periodicity / 1000;   % msec → seconds || time between frames. the better one
NpA = 2000;                         % Np for angles

% Pre‑compute steering matrix  A (Rx × La)
k = 2*pi/lambda;
A  = exp(1j * k * d * (0:Rx-1).' * sind(azGrid));

%% 1.  Build the Angle–Time PMM (A‑PMM) matrix ----------------------------
A_PMM = zeros(La, F);        % [angle × frame]

for f = 1:F
    % --- 1.1  pick UAV‑related range bin (closest integer)
    rBin = round(track_pf_range(f) / Rres) + 1;   % UAV‑range bin
    Xrd  = squeeze(RDcube_cplx(rBin, :, f, :));   % [D × Rx]
    
    % --- 1.2  spatial covariance  (average small Doppler window)
    center = round(D/2);
    dSel = max(1,center-floor(snapWin/2)):min(D,center+floor(snapWin/2));
    snapshots = Xrd(dSel, :).';         % [Rx × |dSel|]
    Rcov = (snapshots * snapshots.') / size(snapshots,2) + delta*eye(Rx);

    %{
    Diagonal loading for stability | excluding this because it's too
    complex. No need for this complexity
    
        delta    = deltaFrac * trace(Rcov)/Rx;
        Rcov     = Rcov + delta*eye(Rx);
    Robust “inverse” via backslash or pinv
        if rcond(Rcov) > 1e-6
            Xa = Rcov \ A;                          % [Rx × La]
        else
            warning('Capon Rcov ill-conditioned (rcond=%.2e): using pinv', rcond(Rcov));
            Xa = pinv(Rcov, svdTol) * A;            % [Rx × La]
        end
    %}
    
    % --- 1.3  Capon weights for every angle (shared)
    % Solve solves Rcov * X = A  →  X = R⁻¹A
    Xa    = Rcov \ A;                 % Rx × La || X = R⁻¹A
    denom = sum(conj(A) .* Xa, 1);    % 1 × La || = a(θ)ᴴ R⁻¹ a(θ)
    W     = Xa ./ denom;              % Rx × La (each col: w(θ))
    
    % --- 1.4  Beamform: angle doppler slice for the UAV
    Y = Xrd * conj(W);                % [D × La]
    
    % --- 1.5  PMM folding per angle
    for a = 1:La
        dopplerSpectrum = abs(Y(:,a)).';             % 1×D, row vector
        bestScore = 0;

        for j = jMin:jMax
            M = floor(D / j);
            if M < 1 
                break; 
            end
            Xcut = dopplerSpectrum(1:j*M);
            Xmat = reshape(Xcut, j, M);                % j × M
            colAvg = sum(Xmat, 2) ./ M;                % j × 1
            score_j = max(colAvg);                     % peak alignment

            bestScore = max(bestScore, score_j);
        end

        A_PMM(a,f) = bestScore;                        % folding result
    end
end

%% 2.  Spectral subtraction over angles (same as range) -------------------
Nvec = mean(A_PMM, 2);                               % eq.(12)
Gain = (Nvec' * A_PMM) / (Nvec' * Nvec);             % 1×F   eq.(13)
Sang = A_PMM - Nvec * Gain;                          % cleaned A‑PMM  eq.(14)

%% 3.  DP tracker in angle domain ----------------------------------------
%   Let max turn‑rate ≈ 30°/s → one‑frame jump ≤ J bins
maxAngVel = 30;
angleRes = diff(azGrid(1:2));                % 1°
J = ceil(maxAngVel * Td / angleRes);         % e.g. 3 bins

thetaA = zeros(La,F);
prevA = zeros(La,F);

thetaA(:,1) = Sang(:,1);

for t = 2:F % for each frame
    for a = 1:La  % for each angle bins
        low = max(1, a-J);  
        high = min(La, a+J);
        [bestVal, idx] = max(thetaA(low:high, t-1));
        thetaA(a,t) = bestVal + Sang(a, t);
        prevA(a,t)  = low + idx - 1;
    end
end

% backtrack
aTrack = zeros(1,F);
[~, aTrack(F)] = max(thetaA(:,F));
for t = F:-1:2
    aTrack(t-1) = prevA(aTrack(t),t); 
end
az_TrackDP = azGrid(aTrack);                       % degrees

%% 4. particle‑filter smoothing (same code as before) -------------
%  (replace range bins with angle bins; J becomes std, etc.)

% State = [angleBin; angularVel_bins_per_frame]
particlesA = zeros(2, NpA);
weightsA   = ones(1, NpA)/NpA;

% init around DP start
particlesA(1,:) = aTrack(1) + randn(1, NpA);
particlesA(2,:) = 0;

% process noise (bins, bins/frame)
sigma_a = 1;      % angle drift
sigma_omega = 0.5;    % angular speed drift

est = zeros(2, F);
est(:,1) = [az_TrackDP(1); 0];

for t = 2:F
  % 3.1 Predict
  particlesA(1,:) = particlesA(1,:) + particlesA(2,:) * Td + sigma_a * randn(1,NpA);
  particlesA(2,:) = particlesA(2,:) + sigma_omega * randn(1,NpA);
  
  % 3.2 Update weights by likelihood ∝ Sang(bin, t)
  bin_idx = round(particlesA(1,:));
  bin_idx = min(max(bin_idx,1), La);
  rawLik  = Sang( bin_idx, t ).';
  likelihood = max(rawLik - min(rawLik), 0) + eps;
  weightsA = weightsA .* likelihood;
  weightsA = weightsA / sum(weightsA);

  % do I need to do sumWA? For normalization like range particle filtering
  
  % 3.3 Resample
  idx = randsample(1:NpA, NpA, true, weightsA);
  particlesA = particlesA(:, idx);
  weightsA   = ones(1,NpA)/NpA;
  
  % 3.4 Estimate
  est(:,t) = particlesA * weightsA.';
end

% Convert to physical units
az_pf    = azGrid( round(est(1,:)) );             % degrees
omega_pf = est(2,:) * angleRes; % / Td;           % deg/s

%% 4) Visualize results
timeAxis = (0:F-1)*Td;

figure;

subplot(2,2,1);
imagesc(timeAxis, azGrid, Sang);
axis xy; hold on;
plot(timeAxis, az_TrackDP, 'w-', 'LineWidth', 2);
plot(timeAxis, az_pf, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Azimuth (°)');
legend('DP','PF');
title('Cleaned Angle–Time PMM & Tracks');

subplot(2,2,2);
plot(timeAxis, az_pf, 'r-', 'LineWidth', 1.5);
hold on;
plot(timeAxis, omega_pf, 'b-', 'LineWidth', 1);
xlabel('Time (s)');
yyaxis right; ylabel('Angular speed (°/s)');
yyaxis left;  ylabel('Azimuth (°)');
title('PF: Azimuth and Angular Velocity');
legend('Azimuth','Angular Speed','Location','best');

subplot(2,2,3);
imagesc(timeAxis, azGrid, Sang);
axis xy; hold on;
plot(timeAxis, az_TrackDP,'-w','LineWidth',2);
xlabel('Time (s)'); ylabel('Azimuth (°)');
title('Angle–Time PMM with DP track');

subplot(2,2,4);
plot(timeAxis, az_TrackDP);
xlabel('Time (s)'); ylabel('Azimuth (°)');
title('Tracked UAV azimuth');

