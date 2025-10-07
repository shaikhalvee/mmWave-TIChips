clear all;

%% Load Data
S = load('output/fft_result_cube.mat');
fft_complex_radar_cube = S.fft_complex_radar_cube;
[R, D, F, Rx] = size(fft_complex_radar_cube);

fft_complex_radar_cube = reshape(fft_complex_radar_cube, R, D, Rx, F);

mmwave_device = S.mmWave_device;

numRx = Rx;
% choose number of FFT points (can zero-pad for finer angle grid)
nAngFFT = numRx;
% nAngFFT = 2^nextpow2(numRx);

% (optional) spatial window to reduce sidelobes
win = hann(numRx);              % numRx×1 Hanning window

% reshape window so it multiplies the 3rd dim
win3d = reshape(win, [1,1,numRx,1]);    
data_win = fft_complex_radar_cube .* win3d;   % [R×D×Rx×F]

% perform FFT along the antenna (3rd) dimension, then fftshift to center zero
angleFFTcube = fftshift( ...
    fft(data_win, nAngFFT, 3), ...   % FFT along dim-3
    3);                               % shift so bin-1 is −Nyquist
% size(angleFFTcube) = [R×D×nAngFFT×F]
angleFFTcubeSize = size(angleFFTcube);

% radar wavelength and element spacing
c      = physconst('Lightspeed');
fc     = mmwave_device.freq;    % in Hz
lambda = c/fc;                              
d      = lambda/2;        % half-λ spacing (typical for ULA)

% spatial‐frequency axis u ∈ [−½, +½)
u = (-nAngFFT/2 : nAngFFT/2-1)/nAngFFT;    

% map to angles (only where |sinθ|≤1)
sin_th = u * (lambda/d);
angleAxis = asind( sin_th );    % in degrees

% pre-allocate
doa_est = nan(R, D, F);

for f = 1:F
  for r = 1:R
    for d = 1:D
      % get the complex angle spectrum at this cell
      spec = squeeze( angleFFTcube(r,d,:,f) );    % [nAngFFT×1]
      % find the strongest bin
      [~, binIdx] = max( abs(spec) );            
      % map to physical angle
      doa_est(r,d,f) = angleAxis(binIdx);        
    end
  end
end

