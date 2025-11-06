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


%max range: 50 meters

%--------------------------------------------------------------------------
% chirpProfile_TxBF_SMRR
%
% This script configures TX beamforming parameters for a short- to medium-
% range radar scenario, with a nominal max range ~50 meters.
%
% Returns a 'params' structure containing:
%   - Antenna geometry selections
%   - Chirp and frame parameters for beamforming
%   - Derived data like range resolution
%--------------------------------------------------------------------------

function [params] = chirpProfile_TxBF_SMRR(angles)

    % TI 4-Chip Cascade board reference
    platform = 'TI_4Chip_CASCADE';
    config_profile = 'SMRR';

    %% Fixed TI cascade board definitions
    % The arrays below specify the azimuth & elevation positions (in half-lambda units)
    % for the 12 possible TX antennas on TI's 4-chip cascade EVM.
    TI_Cascade_TX_position_azi = [11 10 9 32 28 24 20 16 12 8 4 0];
    TI_Cascade_TX_position_ele = [6 4 1 0 0 0 0 0 0 0 0 0];
    TI_Cascade_Antenna_DesignFreq = 76.8;  % Board design frequency in GHz

    % Speed of light, used for range/doppler calculations
    speedOfLight = 3e8;

    %% Configure TX antennas for beamforming
    % We use TX IDs [12..4], i.e., the last 9 TX antenna elements in descending order.
    params.Tx_Ant_Arr_BF = [12:-1:4];
    
    % Look up their actual positions in half-lambda units (for azimuth beamforming)
    params.D_TX_BF = TI_Cascade_TX_position_azi(params.Tx_Ant_Arr_BF);

    % Enable all 4 devices (master + 3 slaves) and all 16 RX channels
    params.RadarDevice = [1 1 1 1];
    params.Rx_Elements_To_Capture = 1:16;

    %% Define beam steering angles
    % We sweep from -30° to +30°, in steps of 2°. 'NumAnglesToSweep' is derived.
    if isempty(angles)
        params.anglesToSteer = -30:2:30;
    else
        params.anglesToSteer = angles;  % Use provided angles if not empty
    end
    params.NumAnglesToSweep = length(params.anglesToSteer);

    %% Chirp/Profile parameters
    % The radar chirp is defined by slope, idle time, ramp end, sampling, etc.
    nchirp_loops = 128;        % how many times each chirp is repeated in a frame
    Num_Frames = 0;           % how many frames we capture
    
    params.Start_Freq_GHz = 77;    % Start frequency in GHz
    % The slope is set to '8.43 * 3' => 25.29 MHz/us total
    params.Slope_MHzperus = 8.43 * 3;
     
    params.Idle_Time_us = 5;       
    params.Tx_Start_Time_us = 0;   
    params.Adc_Start_Time_us = 6;  
    params.Ramp_End_Time_us = 40;  
    params.Sampling_Rate_ksps = 16000;  
    params.Samples_per_Chirp = 512;    
    params.Rx_Gain_dB = 24;          % Received gain in dB

    % range resolution: 0.2 m

    % Frame config parameters
    params.nchirp_loops = nchirp_loops;
    params.Num_Frames = Num_Frames;
    params.Dutycycle = 0.5;   % fraction of (ON / (ON + OFF)) in each chirp cycle
    params.Chirp_Duration_us = (params.Ramp_End_Time_us + params.Idle_Time_us);
    
    % total ADC samples for the entire run (all angles & frames)
    params.NumberOfSamplesPerChannel = ...
        params.Samples_per_Chirp * nchirp_loops * params.NumAnglesToSweep * params.Num_Frames;

    %% Calculate effective spacing factor 'd_BF'
    % This accounts for differences between actual center frequency & board design freq.
    centerFrequency = params.Start_Freq_GHz + ...
        (params.Samples_per_Chirp / params.Sampling_Rate_ksps * params.Slope_MHzperus) / 2;
    d = 0.5 * centerFrequency / TI_Cascade_Antenna_DesignFreq;
    params.d_BF = d;

    %% Advanced frame configuration
    % Decide if beam steering is changed on a per-frame or per-chirp basis.
    % Here, we use frame-based beam steering, so 'Chirp_Frame_BF = 0'.
    params.Chirp_Frame_BF = 0;
    params.numSubFrames = 1;

    if params.Chirp_Frame_BF == 0
        % Frame-based beam steering config
        params.SF1ChirpStartIdx = 0;
        params.SF1NumChirps = 1;
        params.SF1NumLoops = nchirp_loops;
        % The formula below sets how often bursts repeat in time 
        % (converted to registers with *200).
        params.SF1BurstPeriodicity = ...
            (params.Ramp_End_Time_us + params.Idle_Time_us) * nchirp_loops / params.Dutycycle * 200;
        params.SF1ChirpStartIdxOffset = 1;
        params.SF1NumBurst = params.NumAnglesToSweep;
        params.SF1NumBurstLoops = 1;
        % The total subframe period is number of angles * burst periodicity
        params.SF1SubFramePeriodicity = ...
            params.SF1BurstPeriodicity * params.NumAnglesToSweep;
    else
        % If chirp-based beam steering was selected, it would
        % group all angles in each chirp loop. Not used here.
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

    % frame repetition period in ms, used for controlling how often frames start
    params.Frame_Repetition_Period_ms = ...
        params.SF1SubFramePeriodicity / 200 / 1000;

    %% Algorithm parameters
    % For range/doppler processing, we can apply a window, and define the FFT sizes
    params.ApplyRangeDopplerWind = 1;
    params.rangeFFTSize = 2^ceil(log2(params.Samples_per_Chirp));

    %% Derived parameters (optional debugging or referencing)
    chirpRampTime = params.Samples_per_Chirp / (params.Sampling_Rate_ksps / 1e3);
    % The slope(1) is used if it's an array, but here it's just a single value
    chirpBandwidth = params.Slope_MHzperus(1) * chirpRampTime;  
    rangeResolution = speedOfLight / 2 / (chirpBandwidth * 1e6);
    params.rangeBinSize = rangeResolution * params.Samples_per_Chirp / params.rangeFFTSize;
    
    params.f_s = params.Sampling_Rate_ksps * 1e3; % ADC sampling rate (Hz)
    params.maxRange = (params.f_s * speedOfLight) / (2 * params.Slope_MHzperus * 1e12); % meters

    params.T_chirp = params.Chirp_Duration_us * 1e-6; % seconds
    params.lambda = speedOfLight / (params.Start_Freq_GHz * 1e9); % wavelength (m)
    params.velocityResolution = params.lambda / (2 * params.nchirp_loops * params.NumAnglesToSweep * params.T_chirp);
    params.maxVelocity = params.lambda / (4 * params.T_chirp);
    
end
