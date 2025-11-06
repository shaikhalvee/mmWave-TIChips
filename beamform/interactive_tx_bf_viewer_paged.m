function interactive_tx_bf_viewer_paged()
% INTERACTIVE_TX_BF_VIEWER_PAGED: Paged viewer for huge per-frame TXBF results.

    frames_per_batch = 120;
    data_folder = './output/out_txbf_2_11_sl_8_stbl_mov_2/';
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
        'Position', [570 20 80 25], 'Callback', @next_batch);
    hPrev = uicontrol('Style', 'pushbutton', 'String', 'Previous', ...
        'Position', [20 20 80 25], 'Callback', @prev_batch);

    batch_frames = curr_batch_end - curr_batch_start + 1;
    hFrame = uicontrol('Style', 'slider', 'Min', 1, 'Max', batch_frames, ...
        'Value', 1, 'SliderStep', [1/max(1,batch_frames-1), 1/max(1,batch_frames-1)], ...
        'Position', [120 20 350 20]);
    hAngle = uicontrol('Style', 'slider', 'Min', 1, 'Max', nAngles, ...
        'Value', 1, 'SliderStep', [1/max(1,nAngles-1), 1/max(1,nAngles-1)], ...
        'Position', [670 20 350 20]);

    hFrameLabel = uicontrol('Style', 'text', 'Position', [470 20 100 20], ...
        'String', sprintf('Frame: %d/%d', curr_batch_start, total_frames), 'HorizontalAlignment', 'left');
    hAngleLabel = uicontrol('Style', 'text', 'Position', [1020 20 100 20], ...
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

        % Load from batch_data (should only be frames in current batch)
        D = batch_data{frameIdx};
        % RD_map = abs(D.RD_map); % [R D Ang]
        RD_map_abs_sq = (abs(D.RD_map)).^2; % [R D Ang]
        % to_plot = mean(RD_map(:, :, :, angleIdx), 3); % average over Rx
        to_plot = RD_map_abs_sq(:, :, angleIdx);
        range_axis = all_range_axis{globalFrameIdx};
        doppler_axis = all_doppler_axis{globalFrameIdx};

        max_range = 100; % meters
        idx_range = find(range_axis <= max_range);

        to_plot = to_plot(idx_range, :);
        range_axis = range_axis(idx_range);

        range_angle_stich = all_range_angle_stich{globalFrameIdx};

        % RD Map
        axes(hAx1); cla(hAx1);
        if isLog1
            imagesc(doppler_axis, range_axis, 20*log10(to_plot + eps));
            title('Range-Doppler Map (dB)', 'FontSize', 16);
        else
            imagesc(doppler_axis, range_axis, to_plot);
            title('Range-Doppler Map (linear)', 'FontSize', 16);
        end
        xlabel('Doppler (m/s)', 'FontSize', 16); 
        ylabel('Range (m)', 'FontSize', 16);
        colorbar; axis xy;
        set(gca, 'FontSize', 16);
 
        % Zoom region [EDIT as needed]
        % doppler_zoom = [-5 5]; % Doppler axis limits for zoom
        % range_zoom = [95 115]; % Range axis limits for zoom

        % Draw rectangle on main plot
        hold on;
        % rect = rectangle('Position', [doppler_zoom(1), range_zoom(1), diff(doppler_zoom), diff(range_zoom)], ...
            % 'EdgeColor', 'k', 'LineWidth', 2);
        
        % Create inset axes [Position in normalized units: [x y w h]]
        % insetPos = [0.50 0.65 0.3 0.25];
        % insetPos = [0.73 0.5 0.25 0.35]; % [x y w h] in normalized figure units
        % hInset = axes('Position', insetPos);

        % Find the indices corresponding to zoom region
        % [~, x1] = min(abs(doppler_axis - doppler_zoom(1)));
        % [~, x2] = min(abs(doppler_axis - doppler_zoom(2)));
        % [~, y1] = min(abs(range_axis - range_zoom(1)));
        % [~, y2] = min(abs(range_axis - range_zoom(2)));

        % zoom_data = to_plot(y1:y2, x1:x2);
        % zoom_x = doppler_axis(x1:x2);
        % zoom_y = range_axis(y1:y2);

        % imagesc(zoom_x, zoom_y, zoom_data);
        % axis xy tight;
        % colormap(hInset);
        % set(hInset, 'FontSize', 20);
        % box on;
        % title('Zoomed Range Doppler', 'FontSize', 20);

        % 4. Draw connecting lines (use annotation, which needs normalized figure units)
        % Rectangle corners in main axes (convert to normalized units)
        % main_ax_pos = get(hInset, 'Position');
        % % Lower-left and upper-right corners (in axes units)
        % LL = [doppler_zoom(1), range_zoom(1)];
        % UR = [doppler_zoom(2), range_zoom(2)];
        % % Convert these to normalized figure units:
        % ax = hInset;
        % xl = xlim(ax); yl = ylim(ax);
        % xrange = xl(2) - xl(1); yrange = yl(2) - yl(1);

        % ll_norm = [(LL(1)-xl(1))/xrange*main_ax_pos(3)+main_ax_pos(1), ...
        %            (LL(2)-yl(1))/yrange*main_ax_pos(4)+main_ax_pos(2)];
        % ur_norm = [(UR(1)-xl(1))/xrange*main_ax_pos(3)+main_ax_pos(1), ...
                   % (UR(2)-yl(1))/yrange*main_ax_pos(4)+main_ax_pos(2)];

        % Inset corners (e.g., lower left and upper right)
        % inset_ax_pos = get(hInset, 'Position');
        % inset_ll = [inset_ax_pos(1), inset_ax_pos(2)];
        % inset_ur = [inset_ax_pos(1)+inset_ax_pos(3), inset_ax_pos(2)+inset_ax_pos(4)];

        % Draw lines: main LL to inset LL, main UR to inset UR
        % annotation('line', [ll_norm(1), inset_ll(1)], [ll_norm(2), inset_ll(2)], 'Color', 'k', 'LineWidth', 1);
        % annotation('line', [ur_norm(1), inset_ur(1)], [ur_norm(2), inset_ur(2)], 'Color', 'k', 'LineWidth', 1);

        % Optionally draw a rectangle on the main plot to show zoom area
        % axes(gca); % Back to main axes
        % hold on;
        % rectangle('Position', [doppler_zoom(1), range_zoom(1), ...
        %     diff(doppler_zoom), diff(range_zoom)], ...
        %     'EdgeColor', 'w', 'LineWidth', 2, 'LineStyle', '--');
        % hold off;

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
            plot(doppler_axis, 20*log10(max(to_plot, 1)+eps), 'LineWidth', 1.0);
            title('Doppler Profile (dB)', 'FontSize', 16);
            ylabel('Power (dB)', 'FontSize', 16);
        else
            % to_plot_max = squeeze(max(to_plot,[],1));
            % middle_bin = 64;
            % to_plot_max_exc_middle = to_plot_max([1:middle_bin-1 middle_bin+1:end], :); % [127 19]
            % doppler_axis_exc_middle = doppler_axis([1:middle_bin-1 middle_bin+1:end]);  % [127,1]
            % to_plot_max_ang = max(to_plot_max_exc_middle, [], 2);  % [127,1] (max over angles)
            % 
            % plot(doppler_axis_exc_middle, to_plot_max_ang, 'LineWidth', 1.0);
            % % xlabel('Doppler (m/s)');
            % ylabel('Max Power (linear)');
            % title('Doppler Profile (excluding center bin)');


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
        % Build axes (same as your display_graph)
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
        % Release previous batch data explicitly to save memory
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
