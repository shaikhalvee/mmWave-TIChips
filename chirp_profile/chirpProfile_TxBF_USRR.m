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

% max range = 15 meters.

function [params] = chirpProfile_TxBF_USRR(angles)

	% TI cascade board antenna configurations

    % 'platform' variable might be used for reference checks
    platform = 'TI_4Chip_CASCADE';
    config_profile = 'USRR';

    %% Fixed antenna ID and position values for TI 4-chip cascade board
    % These arrays define the azimuth and elevation positions of the 12 TX antennas
    % in half-lambda units.
    % 12 TX antenna azimuth positions on TI 4-chip cascade EVM
    TI_Cascade_TX_position_azi = [11 10 9 32 28 24 20 16 12 8 4 0 ];

    % 12 TX antenna elevation positions on TI 4-chip cascade EVM
    TI_Cascade_TX_position_ele = [6 4 1 0 0 0 0 0 0 0 0 0];

    % The board is designed around 76.8 GHz for its antenna spacing
    TI_Cascade_Antenna_DesignFreq = 76.8;   % (GHz)

    % Speed of light (used for computing range/doppler)
    speedOfLight = 3e8;

    % Here, we specify which TX antennas to use for beamforming.
    % The vector [12:-1:4] means TX IDs 12 down to 4 in descending order.
    params.Tx_Ant_Arr_BF = [12:-1:4]; % TX channel IDs to use for beamforming; all 16 RX channels are enabled by default

    % Retrieve the physical TX positions (in half-lambda units) for those selected antennas
    params.D_TX_BF = TI_Cascade_TX_position_azi(params.Tx_Ant_Arr_BF);

    % The radar device array (for multi-chip cascade). Setting each to '1'
    % means all 4 devices (master + 3 slaves) are enabled.
    params.RadarDevice = [1 1 1 1];     % set 1 all the time

    % By default, we enable all 16 RX channels
    params.Rx_Elements_To_Capture = 1:16;


    %% Define angles to steer in TX beamforming mode (azimuth angles in degrees)
    if isempty(angles)
        params.anglesToSteer = -30:2:30;   % from -30° to +30°, in 2° steps
    else
        params.anglesToSteer = angles;  % Use provided angles if not empty
    end
    params.NumAnglesToSweep = length(params.anglesToSteer);  % number of angles

	%% Chirp/Profile parameters
    % These set up how each chirp is generated, including frequency slope, idle time,
    % ramp time, sampling rate, and more. "nchirp_loops" is how many times each set of
    % chirps is repeated per frame. "Num_Frames" is how many frames to capture in total.
    nchirp_loops = 128;    % number of chirp loops (repetitions)
    Num_Frames = 0;       % number of frames

    params.Start_Freq_GHz = 77;                % Starting frequency for the chirp (GHz)
    params.Slope_MHzperus = 79;                % Frequency slope (MHz/us)
    params.Idle_Time_us  = 5;                  % Idle time per chirp (µs)
    params.Tx_Start_Time_us = 0;               % TX start time offset (µs)
    params.Adc_Start_Time_us = 6;              % ADC start time (µs)
    params.Ramp_End_Time_us = 40;              % Ramp end time (µs)
    params.Sampling_Rate_ksps = 16000;          % Sampling rate (ksps = kilo-samples/second)
    params.Samples_per_Chirp = 512;            % Number of ADC samples taken during each chirp
    params.Rx_Gain_dB = 24;                    % RX gain in dB

    % range resolution: 0.0593m

	%% Frame config
	% Store the loop and frame counts in the params structure
    params.nchirp_loops = nchirp_loops;
    params.Num_Frames = Num_Frames;
	% Duty cycle used in advanced frame configs to space out chirps
	params.Dutycycle = 0.5;                    % (ON duration)/(ON+OFF duration)
	% The total time for one chirp
	params.Chirp_Duration_us = (params.Ramp_End_Time_us + params.Idle_Time_us); % us
	% NumberOfSamplesPerChannel is used to estimate how many ADC samples
    % the capture card will receive per channel.
    params.NumberOfSamplesPerChannel = params.Samples_per_Chirp * nchirp_loops ...
                                           * params.NumAnglesToSweep * params.Num_Frames;
    % above number is the number of ADC samples received per channel. this value is used in HSDC for data capture

	%d = 0.5*actual wavelength/wavelength for antenna design
	%``= 0.5 * actual center frequency/board antenna design frequency
    %% Compute an effective antenna spacing factor (d_BF)
    % to account for differences between the current center frequency and the board's design frequency
    centerFrequency = params.Start_Freq_GHz + ...
        (params.Samples_per_Chirp / params.Sampling_Rate_ksps * params.Slope_MHzperus)/2;

    % d_BF = 0.5 * (centerFrequency / TI_Cascade_Antenna_DesignFreq)
    % If centerFrequency == 76.8, that means d_BF = 0.5
    d = 0.5 * centerFrequency / TI_Cascade_Antenna_DesignFreq;
	params.d_BF = d;


    %% Advanced Frame Configuration
    % "Chirp_Frame_BF" decides if the beam steering is changed on a per-chirp basis (1)
    % or per-frame basis (0).
    params.Chirp_Frame_BF = 0;        % 0 => frame-based beam steering
    params.numSubFrames = 1;         % By default, use 1 sub-frame

	% Sub-frame config for frame-based beam steering:
	if params.Chirp_Frame_BF == 0 % frame based
        params.SF1ChirpStartIdx = 0; %SF1 Start index of the first chirp in this sub frame
        params.SF1NumChirps = 1; % SF1 Number Of unique Chirps per burst
        params.SF1NumLoops = nchirp_loops;% SF1 Number Of times to loop through the unique chirps in each burst
        % multiply 200 to convert the value to be programed to the register %example:2000000=10ms
        params.SF1BurstPeriodicity = (params.Ramp_End_Time_us + params.Idle_Time_us)...
            *nchirp_loops /params.Dutycycle*200;              %example:2000000=10ms
        params.SF1ChirpStartIdxOffset = 1; % SF1 Chirps Start Idex Offset
        params.SF1NumBurst = params.NumAnglesToSweep; % SF1 Number Of Bursts constituting this sub frame
        params.SF1NumBurstLoops = 1; % SF1 Number Of Burst Loops
        params.SF1SubFramePeriodicity = params.SF1BurstPeriodicity*params.NumAnglesToSweep;%SF1 SubFrame Periodicity
	elseif params.Chirp_Frame_BF == 1 % chirp based
        params.SF1ChirpStartIdx = 0;
        params.SF1NumChirps = params.NumAnglesToSweep;
        params.SF1NumLoops = nchirp_loops;
        % multiply 200 to convert the value to be programed to the register  %example:2000000=10ms
        params.SF1BurstPeriodicity = (params.Ramp_End_Time_us +params.Idle_Time_us)...
           *nchirp_loops /params.Dutycycle*200 * params.NumAnglesToSweep ;
        params.SF1ChirpStartIdxOffset = 1;
        params.SF1NumBurst = 1;
        params.SF1NumBurstLoops = 1;
        params.SF1SubFramePeriodicity = params.SF1BurstPeriodicity;
	end

    % Frame repetition time (in ms), used by some data capture tools
    params.Frame_Repetition_Period_ms = params.SF1SubFramePeriodicity / 200 / 1000;


    %% Algorithm / processing parameters
    params.ApplyRangeDopplerWind = 1;                   % Whether to apply windowing for range-Doppler
    params.rangeFFTSize = 2^ceil(log2(params.Samples_per_Chirp));  % FFT size for range


    %% Derived parameters for reference or debugging
    params.chirpRampTime = params.Samples_per_Chirp / (params.Sampling_Rate_ksps/1e3);
    params.chirpBandwidth = params.Slope_MHzperus * params.chirpRampTime;  % in MHz
    params.rangeResolution = speedOfLight / 2 / (params.chirpBandwidth * 1e6);
    params.rangeBinSize = params.rangeResolution * params.Samples_per_Chirp / params.rangeFFTSize;
    params.f_s = params.Sampling_Rate_ksps * 1e3; % ADC sampling rate (Hz)
    params.maxRange = (params.f_s * speedOfLight) / (2 * params.Slope_MHzperus * 1e12); % meters

    params.T_chirp = params.Chirp_Duration_us * 1e-6; % seconds
    params.lambda = speedOfLight / (params.Start_Freq_GHz * 1e9); % wavelength (m)
    params.velocityResolution = params.lambda / (2 * params.nchirp_loops * params.NumAnglesToSweep * params.T_chirp);
    params.maxVelocity = params.lambda / (4 * params.T_chirp);
    
end

