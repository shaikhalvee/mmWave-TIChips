function [rng_fft_cube, range_axis, params] = build_range_only_cube(adcRadarData_txbf, params, dcOffsetRemoval)
% adcRadarData_txbf: [Nsamp, Nchirp, Nrx, Nang]
% rng_fft_cube:      [R,     Nchirp, Nrx, Nang]

Ns   = params.Samples_per_Chirp;
Nk   = params.nchirp_loops;
Nrx  = params.numRX;
Nang = params.NumAnglesToSweep;

fs       = params.Sampling_Rate_ksps * 1e3;
slope    = params.Slope_MHzperus * 1e12; % Hz/s
RFFT     = 2^nextpow2(Ns);
win_fast = hann_local(Ns);

x = adcRadarData_txbf;

% optional DC removal over fast-time
if dcOffsetRemoval
    x = x - mean(x,1);
end

% window & range FFT
x = x .* reshape(win_fast, [],1,1,1);
X = fft(x, RFFT, 1);  % [R, K, Rx, Ang]

% axes
c = physconst('LightSpeed');
range_axis = (0:RFFT-1) * c * fs / (2 * slope * RFFT);

rng_fft_cube = X;

% stash some derivations
params.rangeFFTSize = RFFT;
params.samplingRate = fs;
params.slope        = slope;
params.bandwidth    = slope * RFFT / fs;
end
