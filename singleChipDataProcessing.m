

mmwave_json_file_name = GET_MMWAVE_SETUP_JSON_FILE();
adc_bin_file_name   = GET_ADC_DATA_BIN_FILE();
mmWave_device       = iwr6843Device(adc_bin_file_name, mmwave_json_file_name);

raw_adc_data       = READ_ADC_DATA_BIN_FILE_NEW(adc_bin_file_name, mmWave_device);
% raw_adc_data = [rx_channels x ]

% Extract key parameters:
num_frames         = mmWave_device.num_frame;
chirps_per_frame   = mmWave_device.num_chirp_per_frame;
rx_channels        = mmWave_device.num_rx_chnl;
samples_per_chirp  = mmWave_device.num_sample_per_chirp;
range_res          = mmWave_device.range_res;
v_max              = mmWave_device.v_max;
v_res              = mmWave_device.v_res;

% Reshape into [Rx × samples × chirps × frames]:
adc_data_cube      = reshape(raw_adc_data, rx_channels, samples_per_chirp, chirps_per_frame, num_frames);

% Select only the last Rx channel for analysis:
% adc_channel_data_cube = squeeze(adc_data_cube(rx_channels,:,:,:));

% Window functions:
range_window   = hann(samples_per_chirp);
doppler_window = hann(chirps_per_frame);

% Range & velocity axes:
ranges         = (1:samples_per_chirp-1) * range_res;
velocities     = -v_max : v_res : (v_max - v_res);
range_limit    = 10;                  % meters
range_idx      = ranges <= range_limit;
limited_ranges = ranges(range_idx);

% Pre-allocate storage for normalized FFT:
fft_complex_radar_cube = zeros(samples_per_chirp, chirps_per_frame, num_frames, rx_channels);
fft_norm_radar_cube = zeros(samples_per_chirp, chirps_per_frame, num_frames, rx_channels);

% Process for each channel
for rx = 1:rx_channels
    % Frame-by-frame processing & visualization:
    for frame_index = 1:num_frames
        curr_frame = squeeze(adc_data_cube(rx, : , : , frame_index));

        % DC subtraction per range bin:
        curr_frame = curr_frame - mean(curr_frame, 2);

        % 2D windowing:
        curr_frame = curr_frame .* (range_window * doppler_window');

        % Range FFT then Doppler FFT + shift:
        % range_doppler_fft = fftshift( fft( fft(curr_frame, [], 1), [], 2 ), 2 );
        range_fft = fft(curr_frame, [], 1);
        doppler_fft = fft(range_fft, [], 2);
        range_doppler_fft = fftshift(doppler_fft, 2);
        
        % mag = abs(range_doppler_fft);
        % noise_flr = median(mag(:));
        % norm_fft = mag / noise_flr;

        fft_complex_radar_cube(:, : , frame_index, rx) = range_doppler_fft;
        % fft_norm_radar_cube(:, : , frame_index, rx) = norm_fft;
        
    end
end


% Save for CFAR processing:
save('output/fft_result_cube.mat', 'fft_complex_radar_cube', 'mmWave_device', 'velocities', 'limited_ranges', 'range_idx');
