
%---------------------------------------------------------------------
function params = chirpProfile_TxBF_SingleAngle(degree)
% chirpProfile_TxBF_SingleAngle  LRR profile, but only for a user-supplied 
% azimuth angle.
%
%   params = chirpProfile_TxBF_SingleAngle(degree)
%
%   degree  - Azimuth angle to steer the TX beam to (e.g. -23.45)
%
%   Returns 'params' structure suitable for run_tx_beamforming_capture
%
%   -- All settings use the TI Cascade LRR default, except only one angle. --


    % TI 4-Chip Cascade board reference
    platform = 'TI_4Chip_CASCADE';

    % Antenna geometry: 12 TX positions on TI 4-chip cascade EVM
    TI_Cascade_TX_position_azi = [11 10 9 32 28 24 20 16 12 8 4 0];
    TI_Cascade_TX_position_ele = [6 4 1 0 0 0 0 0 0 0 0 0];
    TI_Cascade_Antenna_DesignFreq = 76.8; % GHz

    % 1. TX selection and geometry
    params.Tx_Ant_Arr_BF = 12:-1:4; % default: 9 azimuth TXs
    params.D_TX_BF = TI_Cascade_TX_position_azi(params.Tx_Ant_Arr_BF);

    % 2. RX and Device config
    params.RadarDevice = [1 1 1 1];       % all 4 devices on
    params.Rx_Elements_To_Capture = 1:16; % all RX enabled

    % 3. Steering angle
    params.anglesToSteer    = degree;
    params.NumAnglesToSweep = numel(params.anglesToSteer);
    params.Chirp_Frame_BF   = 0;      % 0 = frame-based BF

    % 4. LRR chirp/profile parameters
    params.nchirp_loops         = 64;
    params.Num_Frames           = 4;
    params.numSubFrames         = 1;
    params.Start_Freq_GHz       = 77;
    params.Slope_MHzperus       = 7;      % low slope for long range
    params.Idle_Time_us         = 3;
    params.Tx_Start_Time_us     = 0;
    params.Adc_Start_Time_us    = 5;
    params.Ramp_End_Time_us     = 23;
    params.Sampling_Rate_ksps   = 15000;
    params.Samples_per_Chirp    = 256;
    params.Rx_Gain_dB           = 30;
    params.Dutycycle            = 0.5;

    % 5. Derived: Center frequency and spacing scale
    centerFrequency = params.Start_Freq_GHz + ...
        (params.Samples_per_Chirp / params.Sampling_Rate_ksps * params.Slope_MHzperus) / 2;
    params.d_BF = 0.5 * centerFrequency / TI_Cascade_Antenna_DesignFreq;

    % 6. Burst/chirp/frame config for a single angle, frame-based BF
    params.SF1ChirpStartIdx      = 0;
    params.SF1NumChirps          = 1;
    params.SF1NumLoops           = params.nchirp_loops;
    params.SF1BurstPeriodicity   = (params.Ramp_End_Time_us + params.Idle_Time_us) * ...
                                   params.nchirp_loops / params.Dutycycle * 200;
    params.SF1ChirpStartIdxOffset= 1;
    params.SF1NumBurst           = 1;
    params.SF1NumBurstLoops      = 1;
    params.SF1SubFramePeriodicity= params.SF1BurstPeriodicity * params.NumAnglesToSweep;
    params.Frame_Repetition_Period_ms = params.SF1SubFramePeriodicity / 200 / 1e3;

    % 7. Algorithm/FFT params (range/Doppler)
    params.ApplyRangeDopplerWind = 1;
    params.rangeFFTSize = 2^ceil(log2(params.Samples_per_Chirp));

    % 8. Derived range resolution info
    speedOfLight = 3e8;
    chirpRampTime = params.Samples_per_Chirp / (params.Sampling_Rate_ksps / 1e3); % us
    chirpBandwidth = params.Slope_MHzperus * chirpRampTime;  % MHz
    rangeResolution = speedOfLight / 2 / (chirpBandwidth * 1e6); % meters
    params.rangeBinSize = rangeResolution * params.Samples_per_Chirp / params.rangeFFTSize;

end
