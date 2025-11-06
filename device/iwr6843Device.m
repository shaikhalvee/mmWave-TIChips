classdef iwr6843Device
    properties
        %— RF parameters and derived metrics —%
        freq
        lambda 
        aoa_angle 
        num_angle
        chirp_slope
        bandwidth
        chirp_ramp_time
        chirp_idle_time
        chirp_adc_start_time
        frame_periodicity
        num_frame
        num_sample_per_frame
        size_per_frame
        num_sample_per_chirp
        num_chirp_per_frame
        adc_samp_rate
        adc_bits
        dbfs_coeff
        range_max
        range_res
        v_max
        v_res
        rx_chanl_enable
        num_rx_chnl
        num_byte_per_sample
        win_hann
        is_iq_swap 
        is_interleave
    end

    methods
        % 
        function obj = iwr6843Device(adc_data_bin_file, mmwave_setup_json_file)
            % Load JSON and decode:
            sys_param_json = jsondecode(fileread(mmwave_setup_json_file));

            % RF center frequency & wavelength:
            obj.freq   = sys_param_json.mmWaveDevices.rfConfig.rlProfiles.rlProfileCfg_t.startFreqConst_GHz * 1e9;
            obj.lambda = physconst('LightSpeed') / obj.freq;

            % Angle‐of‐arrival grid (0°–180°):
            obj.aoa_angle = 0:1:180;  
            obj.num_angle = length(obj.aoa_angle);

            % ADC format & channel enable mask:
            obj.num_byte_per_sample = 4;
            obj.rx_chanl_enable     = hex2poly(sys_param_json.mmWaveDevices.rfConfig.rlChanCfg_t.rxChannelEn);
            obj.num_rx_chnl         = sum(obj.rx_chanl_enable);

            % Chirp/frame parameters:
            obj.num_sample_per_chirp  = sys_param_json.mmWaveDevices.rfConfig.rlProfiles.rlProfileCfg_t.numAdcSamples;
            obj.num_chirp_per_frame   = sys_param_json.mmWaveDevices.rfConfig.rlFrameCfg_t.numLoops;
            obj.win_hann              = hanning(obj.num_sample_per_chirp);

            % Compute bytes/frame & frame count from file size:
            obj.size_per_frame = obj.num_byte_per_sample * obj.num_rx_chnl * obj.num_sample_per_chirp * obj.num_chirp_per_frame;
            try
                bin_file = dir(adc_data_bin_file);
            catch
                error('Reading Bin file failed')
            end
            obj.num_frame = floor(bin_file.bytes / obj.size_per_frame);
            obj.num_sample_per_frame = obj.num_rx_chnl * obj.num_chirp_per_frame * obj.num_sample_per_chirp;

            % ADC sampling rate & bits:
            obj.adc_samp_rate = sys_param_json.mmWaveDevices.rfConfig.rlProfiles.rlProfileCfg_t.digOutSampleRate/1000;
            if sys_param_json.mmWaveDevices.rfConfig.rlAdcOutCfg_t.fmt.b2AdcBits == 2
                obj.adc_bits = 16;
            end
            obj.dbfs_coeff = - (20*log10(2^(obj.adc_bits-1)) + 20*log10(sum(obj.win_hann)) - 20*log10(sqrt(2)));

            % Bandwidth & timing:
            obj.chirp_slope         = sys_param_json.mmWaveDevices.rfConfig.rlProfiles.rlProfileCfg_t.freqSlopeConst_MHz_usec;
            obj.bandwidth           = obj.chirp_slope * obj.num_sample_per_chirp / obj.adc_samp_rate;
            obj.chirp_ramp_time     = sys_param_json.mmWaveDevices.rfConfig.rlProfiles.rlProfileCfg_t.rampEndTime_usec;
            obj.chirp_idle_time     = sys_param_json.mmWaveDevices.rfConfig.rlProfiles.rlProfileCfg_t.idleTimeConst_usec;
            obj.chirp_adc_start_time = sys_param_json.mmWaveDevices.rfConfig.rlProfiles.rlProfileCfg_t.adcStartTimeConst_usec;
            obj.frame_periodicity   = sys_param_json.mmWaveDevices.rfConfig.rlFrameCfg_t.framePeriodicity_msec;

            % Range & velocity metrics:
            obj.range_max = physconst('LightSpeed') * obj.adc_samp_rate*1e6 / (2*obj.chirp_slope*1e6*1e6);
            obj.range_res = physconst('LightSpeed') / (2*obj.bandwidth*1e6);
            obj.v_max     = obj.lambda / (4*(obj.chirp_ramp_time + obj.chirp_idle_time)/1e6);
            obj.v_res     = obj.lambda / (2*obj.num_chirp_per_frame*(obj.chirp_idle_time+obj.chirp_ramp_time)/1e6);

            % Raw‐data capture flags:
            obj.is_iq_swap   = sys_param_json.mmWaveDevices.rawDataCaptureConfig.rlDevDataFmtCfg_t.iqSwapSel;
            obj.is_interleave = sys_param_json.mmWaveDevices.rawDataCaptureConfig.rlDevDataFmtCfg_t.chInterleave;
        end

        % function []
    end
end
