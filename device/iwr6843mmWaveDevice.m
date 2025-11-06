%% iwr6843mmWaveDevice

classdef iwr6843mmWaveDevice
    % Handles JSON parsing, unit conversion, multi‑file .bin I/O
    properties
        % set up files
        setupJSON        % setup JSON struct (list of .bin files + mmWave JSON path)
        mmwaveJSON       % TI mmWave JSON struct
        
        testBasePath     % name of the file base path of the test files
        testRootPath     % name of the test root file/folder
        binFileNames     % cell array of .bin file paths
        fidBin           % file IDs for each .bin
        numFramePerFile  % frames in each file
        
        % Data placeholders
        adc_raw_bin               % Raw ADC data vector for last read

        % Radar configuration & derived parameters from mmWave JSON
        start_freq                % Center frequency (Hz)
        lambda                    % Wavelength (m)
        aoa_angle                 % AOA search grid (degrees)
        num_angle                 % Number of angles

        chirp_slope               % Frequency slope (MHz/µs)
        bandwidth                 % Chirp bandwidth (MHz)
        chirp_ramp_time           % Ramp time (µs)
        chirp_idle_time           % Idle time between chirps (µs)
        chirp_cycle_time          % Chirp cycle time (µs)
        chirp_adc_start_time      % ADC start time offset (µs)
        frame_periodicity         % Frame period (ms)

        total_frames              % Total frames across all .bin files
        num_sample_per_frame      % Samples per frame (all Rx)
        size_per_frame            % Bytes per frame in .bin
        num_adc_sample_per_chirp  % ADC samples per chirp
        num_chirp_per_frame       % Chirps per frame

        adc_samp_rate             % ADC sampling rate (Msps)
        adc_bits                  % ADC resolution (bits)
        dbfs_coeff                % Digital full-scale correction (dBFS)

        range_max                 % Maximum unambiguous range (m)
        range_res                 % Range resolution (m)
        v_max                     % Maximum unambiguous velocity (m/s)
        v_res                     % Velocity resolution (m/s)

        rx_chanl_enable           % RX channel enable mask (vector)
        num_rx_chnl               % Number of RX channels
        num_byte_per_sample       % Bytes per ADC sample
        range_hann                % Hanning window for ADC samples
        dopp_hann                 % Hanning window for Chirp loops
        num_lanes                 % Number of LVDS lanes

        is_iq_swap                % Flag: IQ swap enabled
        is_non_interleave         % Flag: channel interleaving enabled

    end
    methods
        function obj = iwr6843mmWaveDevice(setupJsonFile)
            % Constructor reads setup JSON, mmWave JSON, and prepares bin files
            obj.setupJSON = jsondecode(fileread(setupJsonFile));
            
            % Read mmWave JSON referenced in setup
            mmwFile = obj.setupJSON.configUsed;
            obj.mmwaveJSON = jsondecode(fileread(mmwFile));

            % Parse mmWave profile, frame, channel, ADC format
            profileConfig = obj.mmwaveJSON.mmWaveDevices.rfConfig.rlProfiles.rlProfileCfg_t;
            frameConfig = obj.mmwaveJSON.mmWaveDevices.rfConfig.rlFrameCfg_t;
            channelConfig = obj.mmwaveJSON.mmWaveDevices.rfConfig.rlChanCfg_t;
            adcFormat = obj.mmwaveJSON.mmWaveDevices.rfConfig.rlAdcOutCfg_t.fmt;
            devDatafmtCfg = obj.mmwaveJSON.mmWaveDevices.rawDataCaptureConfig.rlDevDataFmtCfg_t;

            % RF parameters
            obj.start_freq = profileConfig.startFreqConst_GHz * 1e9;    % f_st
            obj.lambda = physconst('LightSpeed') / obj.start_freq;
            obj.aoa_angle = 0:1:180;
            obj.num_angle = numel(obj.aoa_angle);

            % Samples & chirps
            obj.adc_samp_rate = profileConfig.digOutSampleRate / 1000;  % f_adc
            obj.num_adc_sample_per_chirp = profileConfig.numAdcSamples; % N_adc
            obj.num_chirp_per_frame = frameConfig.numLoops;             % N_ch
            obj.range_hann = hanning(obj.num_adc_sample_per_chirp);
            obj.dopp_hann = hanning(obj.num_chirp_per_frame);

            obj.chirp_slope = profileConfig.freqSlopeConst_MHz_usec;    % S
            obj.bandwidth = obj.chirp_slope * obj.num_adc_sample_per_chirp / obj.adc_samp_rate; % B
            obj.chirp_ramp_time = profileConfig.rampEndTime_usec;
            obj.chirp_idle_time = profileConfig.idleTimeConst_usec;
            obj.chirp_cycle_time = obj.chirp_ramp_time + obj.chirp_idle_time;
            obj.chirp_adc_start_time = profileConfig.adcStartTimeConst_usec;
            obj.frame_periodicity = frameConfig.framePeriodicity_msec;

            % ADC channel config
            obj.num_byte_per_sample = 4;
            rxMask = sscanf(channelConfig.rxChannelEn,'0x%x');
            obj.rx_chanl_enable = bitget(rxMask,1:4);
            obj.num_rx_chnl = sum(obj.rx_chanl_enable);
            laneMask = sscanf(obj.mmwaveJSON.mmWaveDevices.rawDataCaptureConfig.rlDevLaneEnable_t.laneEn, '0x%x');
            obj.num_lanes = sum(bitget(laneMask, 1:4));

            % sample size in each frame
            obj.num_sample_per_frame = obj.num_rx_chnl * obj.num_chirp_per_frame * obj.num_adc_sample_per_chirp;

            % Frame size
            obj.size_per_frame = obj.num_byte_per_sample * obj.num_sample_per_frame;

            % Setup .bin file list and open
            basePath = obj.setupJSON.capturedFiles.fileBasePath;
            obj.testBasePath = basePath;
            [~, obj.testRootPath, ~] = fileparts(basePath);
            files = obj.setupJSON.capturedFiles.files;
            
            % calculate total frames
            cur_frame = 0;
            for i=1:numel(files)
                fn = fullfile(basePath, files(i).processedFileName);
                obj.binFileNames{i} = fn;
                obj.fidBin(i) = fopen(fn,'r');
                finfo = dir(fn);
                frameSize = finfo.bytes / obj.size_per_frame;
                frames = floor(frameSize);
                obj.numFramePerFile(i) = frames;
                cur_frame = cur_frame + frames;
            end
            obj.total_frames = cur_frame;

            % ADC output config
            obj.adc_bits = (adcFormat.b2AdcBits==2)*16 + (adcFormat.b2AdcBits~=2)*adcFormat.b2AdcBits;
            obj.dbfs_coeff = - (20*log10(2^(obj.adc_bits-1)) + 20*log10(sum(obj.range_hann)) - 20*log10(sqrt(2)));

            % Range & velocity calculations
            obj.range_max = physconst('LightSpeed') * obj.adc_samp_rate*1e6 / (2 * obj.chirp_slope*1e6 * 1e6);
            obj.range_res = physconst('LightSpeed') / (2 * obj.bandwidth*1e6);
            obj.v_max = obj.lambda / (4*(obj.chirp_cycle_time)/1e6);
            obj.v_res = obj.lambda / (2*obj.num_chirp_per_frame*(obj.chirp_cycle_time)/1e6);

            % Flags
            obj.is_iq_swap    = devDatafmtCfg.iqSwapSel;
            obj.is_non_interleave = devDatafmtCfg.chInterleave;
        end
        
        
        function rawFrame = readRawFrame(obj, frameIdx)
            % Reads a single frame's raw ADC data from the appropriate .bin
            idx = frameIdx;
            for i=1:numel(obj.numFramePerFile)
                if idx <= obj.numFramePerFile(i)
                    fid = obj.fidBin(i);
                    break;
                end
                idx = idx - obj.numFramePerFile(i);
            end
            offset = (idx-1) * obj.size_per_frame;
            fseek(fid, offset, 'bof');
            
            % Read int16, convert to complex IQ vector - Size 2 bytes 
            % obj.num_sample_per_frame * 2 = obj.size_per_frame / 2;
            % Each frame is 4 bytes with 2 bytes of real data and 2 byte of
            % imaginary data
            rawFrameData = fread(fid, obj.num_sample_per_frame * 2, 'uint16=>single');
            
            % time domain data y value adjustments
            rawDataComplex = rawFrameData - (rawFrameData >=2.^15).* 2.^16; 
            
            if (obj.num_lanes == 2)
                % Convert 2 lane LVDS data to one matrix 
                frameData = obj.reshape2LaneLVDS(rawDataComplex);
            elseif (obj.num_lanes == 4)
                % Convert 4 lane LVDS data to one matrix
                frameData = obj.reshape4LaneLVDS(rawDataComplex);
            end

            if (obj.is_iq_swap == 1)
                % Data is in ReIm format, convert to ImRe format to be used in radarCube 
                frameData(:,[1,2]) = frameData(:,[2,1]);
            end

            % restructuring and forming the complex valued frame
            frameCplx = frameData(:,1) + 1i*frameData(:,2);
            
            frameComplex = single(zeros(obj.num_adc_sample_per_chirp, obj.num_chirp_per_frame, obj.num_rx_chnl));
            temp = reshape(frameCplx, [obj.num_adc_sample_per_chirp * obj.num_rx_chnl, obj.num_chirp_per_frame]).';
            
            for chirp_idx = 1:obj.num_chirp_per_frame
                chirpData_1D = temp(chirp_idx, :); % This is a row vector 1 x (NSample * NChan)
                if (obj.is_non_interleave == 1)
                    % non interleaved
                    slice_NSample_x_NChan = reshape(chirpData_1D, [obj.num_adc_sample_per_chirp, obj.num_rx_chnl]);
                    frameComplex(:, chirp_idx, :) = slice_NSample_x_NChan;
                else
                    % interleaved
                    slice_NChan_x_NSample = reshape(chirpData_1D, [obj.num_rx_chnl, obj.num_adc_sample_per_chirp]);
                    frameComplex(:, chirp_idx, :) = slice_NChan_x_NSample.';
                end
            end
            rawFrame = frameComplex;
            % rawFrame = [R, D, Rx]
        end

        function closeFiles(obj)
            % Close all open file IDs
            for f = obj.fidBin, fclose(f); end
        end

        function frameData = reshape2LaneLVDS(obj, rawData)
            % Convert 2 lane LVDS data to one matrix 
            rawData4 = reshape(rawData, [4, length(rawData)/4]);
            rawDataI = reshape(rawData4(1:2,:), [], 1);
            rawDataQ = reshape(rawData4(3:4,:), [], 1);
    
            frameData = [rawDataI, rawDataQ];
        end

        function frameData = reshape4LaneLVDS(obj, rawData)
            % Convert 4 lane LVDS data to one matrix 
            rawData8 = reshape(rawData, [8, length(rawData)/8]);
            rawDataI = reshape(rawData8(1:4,:), [], 1);
            rawDataQ = reshape(rawData8(5:8,:), [], 1);
    
            frameData= [rawDataI, rawDataQ];
        end
    end
end
