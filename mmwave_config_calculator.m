function mmwave_config_calculator
    % mmwave_config_calculator: Interactive tool for mmWave MIMO derived parameters.
    % Author: ChatGPT for Alvee

    c = 3e8; % Speed of light (m/s)
    
    % ---- Build UI ----
    f = uifigure('Name','mmWave Config Calculator','Position',[300 300 560 480]);

    % -- Input fields --
    % Parameter name, default, position
    inputs = {
        'Start Frequency (GHz)',      77,      [20 420 170 22]
        'Chirp Slope (MHz/us)',       70,      [20 380 170 22]
        'ADC Sample Rate (ksps)',     8000,    [20 340 170 22]
        'Num ADC Samples',            256,     [20 300 170 22]
        'ADC Start Time (us)',        6,       [20 260 170 22]
        'Ramp End Time (us)',         40,      [20 220 170 22]
        'Idle Time (us)',             10,      [20 180 170 22]
        'Num Chirps/Loop',            64,      [20 140 170 22]
        'Num TX Antennas',            3,       [20 100 170 22]
        'Doppler FFT Size',           64,      [20 60 170 22]
    };

    hEdit = gobjects(size(inputs,1),1);
    for i=1:size(inputs,1)
        uilabel(f,'Text',inputs{i,1},'Position',inputs{i,3} + [0 0 120 0]);
        hEdit(i) = uieditfield(f,'numeric','Position',inputs{i,3} + [130 0 70 0],'Value',inputs{i,2});
    end

    % -- Output area --
    hText = uitextarea(f,'Position',[240 60 300 380],'Editable','off','FontSize',13);

    % -- Callback: on edit, update output --
    for i=1:numel(hEdit)
        hEdit(i).ValueChangedFcn = @(src,evt) update();
    end

    % -- Compute and display derived params --
    function update()
        % Read user values
        startFreqGHz     = hEdit(1).Value; % GHz
        chirpSlope       = hEdit(2).Value; % MHz/us
        adcSampleRate    = hEdit(3).Value * 1e3; % ksps -> sps
        numADCSample     = hEdit(4).Value;
        adcStartTime     = hEdit(5).Value; % us
        rampEndTime      = hEdit(6).Value; % us
        idleTime         = hEdit(7).Value; % us
        nchirp_loops     = hEdit(8).Value;
        numTxAnt         = hEdit(9).Value;
        DopplerFFTSize   = hEdit(10).Value;
        
        % Convert units
        startFreqConst   = startFreqGHz * 1e9;    % Hz
        chirpSlopeHz     = chirpSlope * 1e6 / 1e-6; % Hz/s
        chirpSlopeHz_per_us = chirpSlope * 1e6; % Hz/us

        adcStartTimeConst = adcStartTime * 1e-6; % s
        chirpRampEndTime = rampEndTime * 1e-6; % s
        chirpIdleTime    = idleTime * 1e-6; % s

        % ---- Derived parameters (from your formulas) ----
        chirpRampTime       = numADCSample/adcSampleRate; % s
        chirpBandwidth      = chirpSlopeHz_per_us * chirpRampTime; % Hz
        chirpInterval       = chirpRampEndTime + chirpIdleTime; % s
        carrierFrequency    = startFreqConst +  (adcStartTimeConst + chirpRampTime/2)*chirpSlopeHz_per_us; % Hz
        lambda              = c/carrierFrequency; % m
        maximumVelocity     = lambda / (chirpInterval*4); % m/s
        maxRange            = c*adcSampleRate*chirpRampTime/(2*chirpBandwidth); % m
        numSamplePerChirp   = round(chirpRampTime*adcSampleRate);
        rangeFFTSize        = 2^(ceil(log2(numSamplePerChirp)));
        numChirpsPerVirAnt  = nchirp_loops;
        rangeResolution     = c/2/chirpBandwidth; % m
        rangeBinSize        = rangeResolution*numSamplePerChirp/rangeFFTSize;
        velocityResolution  = lambda / (2*nchirp_loops * chirpInterval*numTxAnt); % m/s
        velocityBinSize     = velocityResolution*numChirpsPerVirAnt/DopplerFFTSize;

        % ---- Output, nicely formatted ----
        result = sprintf(['Derived parameters:\n',...
            '--------------------------------------\n',...
            'Chirp Ramp Time:      %.3e s\n',...
            'Chirp Bandwidth:      %.3f MHz\n',...
            'Chirp Interval:       %.3e s\n',...
            'Carrier Frequency:    %.3f GHz\n',...
            'Lambda:               %.3f mm\n',...
            'Max Velocity:         %.3f m/s\n',...
            'Max Range:            %.2f m\n',...
            'Num Samples/Chirp:    %d\n',...
            'Range FFT Size:       %d\n',...
            'Num Chirps/Virt Ant:  %d\n',...
            'Range Resolution:     %.3f m\n',...
            'Range Bin Size:       %.3f m\n',...
            'Velocity Resolution:  %.3f m/s\n',...
            'Velocity Bin Size:    %.3f m/s\n'],...
            chirpRampTime,chirpBandwidth/1e6,chirpInterval,carrierFrequency/1e9,lambda*1e3,...
            maximumVelocity,maxRange,numSamplePerChirp,rangeFFTSize,numChirpsPerVirAnt,...
            rangeResolution,rangeBinSize,velocityResolution,velocityBinSize);

        hText.Value = result;
    end

    update(); % Initialize with default values

end
