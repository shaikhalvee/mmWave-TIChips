function interactive_txbf_viewer_chunked()
% INTERACTIVE_TX_BF_VIEWER_PAGED: Paged viewer for huge per-frame TXBF results,
% with optional summing/averaging over multiple frames (frameChunk mode).

    frames_per_batch = 120;
    frameChunk = 8; % <-- Number of consecutive frames to sum/average
    
    data_folder = './output/out_txbf_6_8_sl_22_fr_200/';
    frame_folder = [data_folder 'rangeDopplerFFTmap_10/'];
    config_folder = data_folder;

    % Get all frame file names
    frame_files = dir(fullfile(frame_folder, 'frame_*.mat'));
    total_frames = numel(frame_files);

    % Load metadata arrays (axes, stich, params)
    config_data = load(fullfile(config_folder, 'config.mat'));
    all_range_axis = config_data.all_range_axis;
    all_doppler_axis = config_data.all_doppler_axis;
    all_range_angle_stich = config_data.all_range_angle_stich;

    params_file = dir(fullfile(config_folder, '*_params.mat'));
    assert(~isempty(params_file), 'Cannot find *_params.mat in the given folder');
    params = load(fullfile(config_folder, params_file(1).name), 'params');
    params = params.params;
    anglesToSteer = params.anglesToSteer;
    nAngles = params.NumAnglesToSweep;

    % Batch state
    curr_batch_start = 1;
    curr_batch_end = min(frames_per_batch, total_frames);

    % Pre-allocate to minimize workspace memory use
    batch_data = {};

    % UI setup
    hFig = figure('Name', 'TX Beamforming Interactive Viewer (Paged)', 'NumberTitle', 'off', 'Position', [100 100 1200 800]);
    hNext = uicontrol('Style', 'pushbutton', 'String', 'Next', ...
        'Position', [550 10 80 25], 'Callback', @next_batch);
    hPrev = uicontrol('Style', 'pushbutton', 'String', 'Previous', ...
        'Position', [20 10 80 25], 'Callback', @prev_batch);

    batch_frames = curr_batch_end - curr_batch_start + 1;
    hFrame = uicontrol('Style', 'slider', 'Min', 1, 'Max', batch_frames, ...
        'Value', 1, 'SliderStep', [1/max(1,batch_frames-1), 1/max(1,batch_frames-1)], ...
        'Position', [120 10 350 20]);
    hAngle = uicontrol('Style', 'slider', 'Min', 1, 'Max', nAngles, ...
        'Value', 1, 'SliderStep', [1/max(1,nAngles-1), 1/max(1,nAngles-1)], ...
        'Position', [650 10 350 20]);

    hFrameLabel = uicontrol('Style', 'text', 'Position', [470 10 80 20], ...
        'String', sprintf('Frame: %d/%d', curr_batch_start, total_frames), 'HorizontalAlignment', 'left');
    hAngleLabel = uicontrol('Style', 'text', 'Position', [1000 10 80 20], ...
        'String', sprintf('Angle: %d°', anglesToSteer(1)), 'HorizontalAlignment', 'left');

    hAx1 = subplot(2,2,1);
    hCB1 = uicontrol('Style', 'checkbox', 'String', 'Log (dB)', ...
        'Units', 'normalized', 'Position', [0.11 0.51 0.05 0.03], 'Value', 1, 'Parent', hFig);
    hAx2 = subplot(2,2,2);
    hCB2 = uicontrol('Style', 'checkbox', 'String', 'Log (dB)', ...
        'Units', 'normalized', 'Position', [0.55 0.51 0.05 0.03], 'Value', 1, 'Parent', hFig);
    hAx3 = subplot(2,2,3);
    hCB3 = uicontrol('Style', 'checkbox', 'String', 'Log (dB)', ...
        'Units', 'normalized', 'Position', [0.11 0.04 0.05 0.03], 'Value', 1, 'Parent', hFig);
    hAx4 = subplot(2,2,4);

    % Load first batch
    load_current_batch();

    % -- Callbacks --
    set(hFrame, 'Callback', @updatePlots);
    set(hAngle, 'Callback', @updatePlots);
    set(hCB1, 'Callback', @updatePlots);
    set(hCB2, 'Callback', @updatePlots);
    set(hCB3, 'Callback', @updatePlots);

    function updatePlots(~,~)
        frameIdx = round(get(hFrame, 'Value'));
        angleIdx = round(get(hAngle, 'Value'));
        isLog1 = get(hCB1, 'Value');
        isLog2 = get(hCB2, 'Value');
        isLog3 = get(hCB3, 'Value'); 

        % Global frame index
        globalFrameIdx = curr_batch_start + frameIdx - 1;
        set(hFrameLabel, 'String', sprintf('Frame: %d/%d', globalFrameIdx, total_frames));
        set(hAngleLabel, 'String', sprintf('Angle: %d°', anglesToSteer(angleIdx)));

        % ----------- Multi-frame summation/averaging ----------
        % Determine chunking range (centered at current frame)
        halfChunk = floor(frameChunk / 2);
        startFrame = max(globalFrameIdx - halfChunk, 1);
        endFrame = min(globalFrameIdx + (frameChunk - halfChunk - 1), total_frames);

        sum_RD = 0; count_RD = 0;
        for idx = startFrame:endFrame
            % If this frame is within the current batch, use batch_data (efficient); else, load directly
            if idx >= curr_batch_start && idx <= curr_batch_end
                D = batch_data{idx - curr_batch_start + 1};
            else
                D = load(fullfile(frame_files(idx).folder, frame_files(idx).name));
            end
            RD_map = abs(D.RD_map); % [R D Rx Ang]
            % Average over Rx for the desired angle
            to_add = mean(RD_map(:, :, :, angleIdx), 3);
            sum_RD = sum_RD + to_add;
            count_RD = count_RD + 1;
        end
        to_plot = sum_RD / count_RD;  % Use sum_RD if you want sum, not average

        range_axis = all_range_axis{globalFrameIdx};
        doppler_axis = all_doppler_axis{globalFrameIdx};
        max_range = 200; % meters
        idx_range = find(range_axis <= max_range);
        to_plot = to_plot(idx_range, :);
        range_axis = range_axis(idx_range);

        range_angle_stich = all_range_angle_stich{globalFrameIdx};

        % RD Map
        axes(hAx1); cla(hAx1);
        if isLog1
            imagesc(doppler_axis, range_axis, 20*log10(to_plot + eps));
            title(sprintf('Range-Doppler Map (dB, frames %d-%d)', startFrame, endFrame), 'FontSize', 16);
        else
            imagesc(doppler_axis, range_axis, to_plot);
            title(sprintf('Range-Doppler Map (linear, frames %d-%d)', startFrame, endFrame), 'FontSize', 16);
        end
        xlabel('Doppler (m/s)', 'FontSize', 16); 
        ylabel('Range (m)', 'FontSize', 16);
        colorbar; axis xy;
        set(gca, 'FontSize', 16);

        % Range Profile
        axes(hAx2); cla(hAx2);
        if isLog2
            plot(range_axis, 20*log10(max(to_plot,2)+eps), 'LineWidth', 1.0);
            title('Range Profile (dB)', 'FontSize', 16);
            ylabel('Power (dB)', 'FontSize', 16);
        else
            plot(range_axis, mean(to_plot,2), 'LineWidth', 1.0);
            title('Range Profile (linear)', 'FontSize', 16);
            ylabel('Power (linear)', 'FontSize', 16);
        end
        xlabel('Range (m)', 'FontSize', 16);
        set(gca, 'FontSize', 14);

        % Doppler Profile
        axes(hAx3); cla(hAx3);
        if isLog3
            plot(doppler_axis, log10(max(to_plot,1)+eps), 'LineWidth', 1.0);
            title('Doppler Profile (dB)', 'FontSize', 16);
            ylabel('Power (dB)', 'FontSize', 16);
        else
            plot(doppler_axis, mean(to_plot,1), 'LineWidth', 1.0);
            title('Doppler Profile (linear)', 'FontSize', 16);
            ylabel('Power (linear)', 'FontSize', 16);
        end
        xlabel('Velocity (m/s)', 'FontSize', 16);
        set(gca, 'FontSize', 16);

        % Range-Azimuth
        axes(hAx4); cla(hAx4);
        % Assume range_angle_stich: (range, angle) or (range, angle, ...)
        if ~ismatrix(range_angle_stich)
            range_angle_stich_2d = squeeze(range_angle_stich(:,:,1));
        else
            range_angle_stich_2d = range_angle_stich;
        end
        indices_1D = 1:numel(range_axis);
        sine_theta = sind(anglesToSteer);
        cos_theta = sqrt(1-sine_theta.^2);
        [R_mat, sine_theta_mat] = meshgrid(range_axis(indices_1D), sine_theta);
        [~, cos_theta_mat] = meshgrid(range_axis(indices_1D), cos_theta);
        x_axis = R_mat.*cos_theta_mat;
        y_axis = R_mat.*sine_theta_mat;
        range_angle_stich_2d = (range_angle_stich_2d(indices_1D,:).');
        surf(y_axis, x_axis, abs(range_angle_stich_2d).^0.2,'EdgeColor','none');
        view(0, 60);
        xlabel('meters'); ylabel('meters');
        title('Stich range/azimuth');
        axis tight;
        colorbar;
    end

    % Batch loader: clears previous batch_data to free memory
    function load_current_batch()
        batch_data = cell(1, curr_batch_end-curr_batch_start+1);
        for i = 1:(curr_batch_end-curr_batch_start+1)
            frame_idx = curr_batch_start + i - 1;
            tmp = load(fullfile(frame_files(frame_idx).folder, frame_files(frame_idx).name));
            batch_data{i} = tmp;
        end
        % Reset frame slider max/min to new batch size
        batch_frames = curr_batch_end - curr_batch_start + 1;
        set(hFrame, 'Min', 1, 'Max', batch_frames, 'Value', 1, ...
            'SliderStep', [1/max(1,batch_frames-1), 1/max(1,batch_frames-1)]);
        updatePlots();
    end

    function next_batch(~,~)
        if curr_batch_end < total_frames
            curr_batch_start = curr_batch_start + frames_per_batch;
            curr_batch_end = min(curr_batch_start + frames_per_batch - 1, total_frames);
            load_current_batch();
        end
    end

    function prev_batch(~,~)
        if curr_batch_start > 1
            curr_batch_start = max(1, curr_batch_start - frames_per_batch);
            curr_batch_end = min(curr_batch_start + frames_per_batch - 1, total_frames);
            load_current_batch();
        end
    end

    % Initial plot
    updatePlots();

end
