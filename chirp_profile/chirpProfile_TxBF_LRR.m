%  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
%
%
%   Redistribution and use in source and binary forms, with or without
%   modification, are permitted provided that the following conditions
%   are met:
%
%     Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%
%     Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the
%     documentation and/or other materials provided with the
%     distribution.
%
%     Neither the name of Texas Instruments Incorporated nor the names of
%     its contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.
%
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
%   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
%   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
%   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
%   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
%   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
%   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
%

% max range: 300 meters

%--------------------------------------------------------------------------
% chirpProfile_TxBF_LRR
%
% This script configures TX beamforming parameters for a longer-range radar
% scenario, with a nominal max range ~300 meters.
%
% Returns a 'params' structure containing:
%   - Antenna geometry selections
%   - Chirp and frame parameters for beamforming
%   - Derived data like range resolution
%--------------------------------------------------------------------------

function [params] = chirpProfile_TxBF_LRR(angles)

    if isempty(angles)
        params.anglesToSteer = -30:2:30;
    else
        params.anglesToSteer = angles; % Use provided angles if not empty
    end

    % TI 4-Chip Cascade board reference
    platform = 'TI_4Chip_CASCADE';
    config_profile = 'LRR';

    %% Fixed TI cascade board definitions
    TI_Cascade_TX_position_azi = [11 10 9 32 28 24 20 16 12 8 4 0];
    TI_Cascade_TX_position_ele = [6 4 1 0 0 0 0 0 0 0 0 0];
    TI_Cascade_Antenna_DesignFreq = 76.8; % in GHz
    speedOfLight = 3e8;

    %% TX Beamforming antenna selection
    % We'll pick TX channels [12..4], same as in SMRR/USRR, just with different slope and timing for LRR.
    params.Tx_Ant_Arr_BF = 12:-1:4;
    params.D_TX_BF = TI_Cascade_TX_position_azi(params.Tx_Ant_Arr_BF);

    % Enable all 4 devices and all 16 RX channels
    params.RadarDevice = [1 1 1 1];
    params.Rx_Elements_To_Capture = 1:16;

    %% Beam steering angles
    % params.anglesToSteer = [-30:2:30];
    params.NumAnglesToSweep = length(params.anglesToSteer);

    %% Chirp/Profile parameters for LRR
    % Typically, LRR uses lower slope => smaller bandwidth => can detect out to ~300m
    nchirp_loops = 128;
    Num_Frames = 0;
    
    params.Start_Freq_GHz = 77;        % start freq (GHz)
    params.Slope_MHzperus = 7;         % slope is 7 MHz/us (narrower BW than SMRR/USRR)
    params.Idle_Time_us = 5;
    params.Tx_Start_Time_us = 0;
    params.Adc_Start_Time_us = 5;
    params.Ramp_End_Time_us = 40;      % relatively short ramp => longer max range
    params.Sampling_Rate_ksps = 16000; % 15 Msps
    params.Samples_per_Chirp = 512;    
    params.Rx_Gain_dB = 52;

    % range resolution: 1.255 m
    
    % Frame info
    params.nchirp_loops = nchirp_loops;
    params.Num_Frames = Num_Frames;
    params.Dutycycle = 0.5;
    params.Chirp_Duration_us = params.Ramp_End_Time_us + params.Idle_Time_us;
    
    params.NumberOfSamplesPerChannel = ...
        params.Samples_per_Chirp * nchirp_loops * params.NumAnglesToSweep * params.Num_Frames;

    %% Effective spacing factor
    centerFrequency = params.Start_Freq_GHz + ...
        (params.Samples_per_Chirp / params.Sampling_Rate_ksps * params.Slope_MHzperus) / 2;
    d = 0.5 * centerFrequency / TI_Cascade_Antenna_DesignFreq;
    params.d_BF = d;

    %% Advanced frame config
    params.Chirp_Frame_BF = 0;  % frame-based beam steering
    params.numSubFrames = 1;

    if params.Chirp_Frame_BF == 0
        params.SF1ChirpStartIdx = 0;
        params.SF1NumChirps = 1;
        params.SF1NumLoops = nchirp_loops;
        params.SF1BurstPeriodicity = ...
            (params.Ramp_End_Time_us + params.Idle_Time_us) * nchirp_loops / params.Dutycycle * 200;
        params.SF1ChirpStartIdxOffset = 1;
        params.SF1NumBurst = params.NumAnglesToSweep;
        params.SF1NumBurstLoops = 1;
        params.SF1SubFramePeriodicity = ...
            params.SF1BurstPeriodicity * params.NumAnglesToSweep;
    else
        % chirp-based alternative if desired
        params.SF1ChirpStartIdx = 0;
        params.SF1NumChirps = params.NumAnglesToSweep;
        params.SF1NumLoops = nchirp_loops;
        params.SF1BurstPeriodicity = ...
            (params.Ramp_End_Time_us + params.Idle_Time_us) * nchirp_loops / params.Dutycycle * 200 * params.NumAnglesToSweep;
        params.SF1ChirpStartIdxOffset = 1;
        params.SF1NumBurst = 1;
        params.SF1NumBurstLoops = 1;
        params.SF1SubFramePeriodicity = params.SF1BurstPeriodicity;
    end
    
    % Frame repetition time in ms
    % params.Frame_Repetition_Period_ms = params.SF1SubFramePeriodicity / 200 / 1000;
    params.Frame_Repetition_Period_ms = 200;

    %% Algorithm / FFT parameters
    params.ApplyRangeDopplerWind = 1;
    params.rangeFFTSize = 2^ceil(log2(params.Samples_per_Chirp));

    %% Derived parameters
    chirpRampTime = params.Samples_per_Chirp / (params.Sampling_Rate_ksps / 1e3);
    chirpBandwidth = params.Slope_MHzperus(1) * chirpRampTime;  % in MHz
    params.rangeResolution = speedOfLight / 2 / (chirpBandwidth * 1e6);
    params.rangeBinSize = params.rangeResolution * params.Samples_per_Chirp / params.rangeFFTSize;
    params.f_s = params.Sampling_Rate_ksps * 1e3; % ADC sampling rate (Hz)
    params.maxRange = (params.f_s * speedOfLight) / (2 * params.Slope_MHzperus * 1e12); % meters

    params.T_chirp = params.Chirp_Duration_us * 1e-6; % seconds
    params.lambda = speedOfLight / (params.Start_Freq_GHz * 1e9); % wavelength (m)
    params.velocityResolution = params.lambda / (2 * params.nchirp_loops * params.NumAnglesToSweep * params.T_chirp);
    params.maxVelocity = params.lambda / (4 * params.T_chirp);
    
end
