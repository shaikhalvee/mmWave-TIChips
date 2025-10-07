%% File: ProcessingPipeline.m

classdef ProcessingPipeline
    % Encapsulates 1D range-FFT, 2D Range–Doppler FFT, and data export
    properties
        device            % MMWaveDevice instance
    end
    methods
        function obj = ProcessingPipeline(device)
            obj.device = device;
        end

        function [rangeFFT, rdFFT] = computeRangeDoppler(obj, rawFrame)
            % rawFrame: [R × D × Rx]
            [R, D, Rx] = size(rawFrame);

            % zero-pad range to next-power-of-2
            NfftR = 2^nextpow2(R);

            % pre-allocate
            rangeFFT = zeros(NfftR, D, Rx);
            rdFFT    = zeros(NfftR, D, Rx);

            % build 2-D window: hanning in range, hanning in Doppler (chirps)
            winRange   = obj.device.range_hann;      % [R×1]
            winDoppler = obj.device.dopp_hann;       % [D×1]
            win2D      = winRange * winDoppler.';  % [R×D]

            for rx = 1:Rx
                % pull out each Rx’s [R×D] block
                data2D = squeeze(rawFrame(:,:,rx)); % [RxD]

                % % DC subtraction
                data2D = data2D - mean(data2D, 2);

                % apply 2D window
                dataWin = data2D .* win2D;

                % Range FFT along dim-1
                rangeFFT(:,:,rx) = fft(dataWin, NfftR, 1);

                % Doppler FFT along dim-2 + shift zero-Doppler to center
                rdFFT(:,:,rx) = fftshift( fft(rangeFFT(:,:,rx), [], 2), 2 );
            end
        end

        function exportAll(obj, saveRaw, save1D, save2D)
            mmWaveDevice = obj.device;
            F = mmWaveDevice.total_frames;
            R = 2^nextpow2(mmWaveDevice.num_adc_sample_per_chirp);
            D = mmWaveDevice.num_chirp_per_frame;
            X = mmWaveDevice.num_rx_chnl;

            % make output folder
            outputDir = fullfile('output', obj.device.testRootPath);
            if ~exist(outputDir,'dir')
                mkdir(outputDir);
            end

            % build full paths
            rawPath   = fullfile(outputDir, saveRaw);
            fft1DPath = fullfile(outputDir, save1D);
            rd2DPath  = fullfile(outputDir, save2D);

            %% 1) If an old RD-FFT file exists, remove it
            if exist(rd2DPath, 'file')
                delete(rd2DPath);
            end

            %% 2) Pre-create the new MAT-file with v7.3, correct types & metadata
            rangeDopplerFFTData = complex(zeros(R, D, X, F, 'single'));  %# allocate single cube
            

            % this save both creates the file and sets up the variables with proper types
            save(rd2DPath, ...
                'rangeDopplerFFTData', ...
                'mmWaveDevice', ...
                '-v7.3');

            %% 3) Re-open it for streaming writes
            mf = matfile(rd2DPath, 'Writable', true);

            % prepare your RAW and 1D-FFT containers
            adcBinRaw    = cell(1, F);
            rangeFFTData = cell(1, F);

            % loop over frames
            for f = 1:F
                rawFrame = obj.device.readRawFrame(f);
                adcBinRaw{f} = rawFrame;

                [rangeFFTFrame, rdFFTFrame] = obj.computeRangeDoppler(rawFrame);
                rangeFFTData{f} = rangeFFTFrame;

                % write just this slice into the on-disk single cube
                mf.rangeDopplerFFTData(:,:,:,f) = single(rdFFTFrame);
            end

            %% 4) Save the RAW & 1D-FFT cell arrays in the same folder
            if ~isempty(saveRaw)
                save(rawPath,   'adcBinRaw',    '-v7.3');
            end
            if ~isempty(save1D)
                save(fft1DPath, 'rangeFFTData', '-v7.3');
            end
        end
    end
end
