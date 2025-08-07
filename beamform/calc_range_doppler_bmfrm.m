function [RD_map, range_axis, ...
    doppler_axis, range_angle_stich, ...
    paramsConfig] = calc_range_doppler_bmfrm(adcRadarData_txbf, ...
        paramsConfig, BF_MIMO_ref, dcOffsetRemoval, dopplerClutterRemoval)
    
    % dim: adc_cube [numRangeBin, numDopplerBin, numRx, numSweepAng]

    % clutter & noise handle
    % dcOffsetRemoval = true;
    % dopplerClutterRemoval = true;

    % var initialize
    paramsConfig.rangeFFTSize = 2^ceil(log2(paramsConfig.Samples_per_Chirp));
    paramsConfig.dopplerFFTSize = 2^ceil(log2(paramsConfig.nchirp_loops));
    
    numRangeBin = paramsConfig.rangeFFTSize;
    numDopplerBin = paramsConfig.dopplerFFTSize;
    d = paramsConfig.d_BF;
    numSweep = paramsConfig.NumAnglesToSweep;
    D_RX = paramsConfig.D_RX;
    % indices_1D = (startRangeInd: params.Samples_per_Chirp-lastRangeIndToThrow) ;

    % old code
    % window_1D = repmat(hann_local(num_samples), 1, num_chirps);
    % window_2D = (repmat(hann_local(numDopplerBin), 1, numRangeBin)).';
    % for Rxnum=1:length(Rx_Ant_Arr)
    %     for sweep=1:num_sweep
    %         radar_data_frame = squeeze(BF_data(:,:,Rxnum,sweep));        
    % 
    %         rangeFFT1(:,:,Rxnum,sweep) = fft(radar_data_frame.*window_1D,rangeFFTSize,1);
    %         rangeDopFFT1(:,:,Rxnum,sweep) = fftshift(fft(rangeFFT1(:,:,Rxnum,sweep).*window_2D,DopplerFFTSize,2),2);
    %         rangeDopFFT1(:,:,Rxnum,sweep) = rangeDopFFT1(:,:,Rxnum,sweep).*(exp(-1i*BF_MIMO_ref(Rxnum)*pi/180));
    %     end
    % end


    % Windowing
    num_samples = paramsConfig.Samples_per_Chirp;
    num_chirps  = paramsConfig.nchirp_loops;
    num_rx      = paramsConfig.numRX;

    range_win   = hann_local(num_samples);
    doppler_win = hann_local(num_chirps);

    % DC Offset removal
    if dcOffsetRemoval
        % Subtract mean across the first dimension (samples)
        adcRadarData_txbf = adcRadarData_txbf - mean(adcRadarData_txbf, 1);
        % adcRadarData_txbf = bsxfun(@minus, adcRadarData_txbf, mean(adcRadarData_txbf,1));
    end

    % apply range-domain windowing
    radar_data_win = adcRadarData_txbf .* reshape(range_win,[],1,1,1);
    
    % Range FFT (dim 1)
    range_fft    = fft(radar_data_win, paramsConfig.rangeFFTSize, 1);  % [range, chirps, Rx, angles]

    % Doppler window coefficient
    range_fft_win = range_fft .* reshape(doppler_win,1,[],1,1);

    % Doppler clutter removal
    if dopplerClutterRemoval
        % Remove mean across slow time (chirps) for each [range, Rx, angle]
        range_fft_win = range_fft_win - mean(range_fft_win, 2);
        % range_fft_win = bsxfun(@minus, range_fft_win, mean(range_fft_win,2));

        % hpFilt = fir1(8, 0.05, 'high'); % FIR high-pass (fir1) filter
        % hpFilt = [1, -2, 1]; % 3-pulse canceller
        % range_fft_win = doppler_highpass_filter(range_fft_win, hpFilt);

        % Replace with mean
        % range_fft_win = range_fft_win - mean(range_fft_win, 2)/2;
    end
    
    % Doppler FFT (dim 2)
    doppler_fft   = fftshift(fft(range_fft_win, numDopplerBin, 2), 2);

    % RX calibration
    for rx = 1:num_rx
        doppler_fft(:,:,rx,:) = doppler_fft(:,:,rx,:) * exp(-1i * BF_MIMO_ref(rx) * pi/180);
    end

    % doppler_fft = [R, D, Rx, numAng] 
    % Combine RX (max or sum, single angle so no angle FFT)
    RD_map = doppler_fft;
    % RD_map = squeeze(mean(abs(doppler_fft), 3)); % [range, doppler], for 1 ang, for multiple [range, doppler, numAng]

    % Axes
    c = physconst('LightSpeed');
    fs = paramsConfig.Sampling_Rate_ksps * 1e3;
    slope = paramsConfig.Slope_MHzperus * 1e12;
    paramsConfig.bandwidth = slope * paramsConfig.rangeFFTSize / fs;
    range_axis = (0:paramsConfig.rangeFFTSize-1) * c * fs / (2 * slope * paramsConfig.rangeFFTSize);
    % PRF = 1e6 / (paramsConfig.Idle_Time_us + paramsConfig.Ramp_End_Time_us);
    PRT = (paramsConfig.Idle_Time_us + paramsConfig.Ramp_End_Time_us) * 1e-6;
    lambda = c / (paramsConfig.Start_Freq_GHz * 1e9);
    v_max = lambda / (4 * PRT);
    % v_max = c * paramsConfig.Slope_MHzperus * 1e6 / (2 * paramsConfig.Start_Freq_GHz * 1e9);
    doppler_axis = linspace(-v_max, v_max, paramsConfig.nchirp_loops);
    paramsConfig.PRT = PRT;
    paramsConfig.lambda = lambda;
    paramsConfig.v_max = v_max;
    paramsConfig.c = c;
    paramsConfig.samplingRate = fs;
    paramsConfig.slope = slope;

    % perform beamsteering towards the angle TX steering angles
    rangeDopplerFFT_zeroDopp = squeeze(doppler_fft(:, paramsConfig.dopplerFFTSize/2+1,:,:));
    range_angle_stich = complex(zeros(numRangeBin, numSweep));
    for angle = 1:numSweep
        angTx = paramsConfig.anglesToSteer(angle);
        wx = sind(angTx);
        az = exp(1*1i*2*pi*d*(D_RX*wx));

        for range = 1:numRangeBin
            RX_data = squeeze(rangeDopplerFFT_zeroDopp(range,:,angle));
            range_angle_stich(range, angle) = az*(RX_data'*RX_data)*az';
        end
    end
end
