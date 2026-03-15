function interactive_tx_bf_viewer()
% INTERACTIVE_TX_BF_VIEWER: Interactive viewer for TX beamforming post-processing results.

    % clearvars -except data_folder;
    %
    % if nargin < 1
    %     data_folder = uigetdir(pwd, 'Select output folder containing the saved MAT files');
    % end

    data_folder = './output/in_txbf_corwalk2/';
    % Load data
    d = dir(fullfile(data_folder, 'rangeDopplerFFTmap.mat'));
    assert(~isempty(d), 'Cannot find rangeDopplerFFTmap.mat in the given folder');
    load(fullfile(data_folder, d(1).name), ...
        'all_to_plot', 'all_range_axis', 'all_doppler_axis', 'all_range_angle_stich');

    d2 = dir(fullfile(data_folder, '*_params.mat'));
    assert(~isempty(d2), 'Cannot find *_params.mat in the given folder');
    load(fullfile(data_folder, d2(1).name), 'params');

    nFrames = numel(all_to_plot); % params.Num_Frames
    nAngles = params.NumAnglesToSweep;
    anglesToSteer = params.anglesToSteer;
    if nAngles == 1 && size(all_to_plot{1}, 4) == 1
        nAngles = 1;
    end

    % Figure
    hFig = figure('Name', 'TX Beamforming Interactive Viewer', 'NumberTitle', 'off', 'Position', [100 100 1200 800]);
    
    % Sliders
    hFrame = uicontrol('Style', 'slider', 'Min', 1, 'Max', nFrames, ...
        'Value', 1, 'SliderStep', [1/(nFrames-1) 1/(nFrames-1)], ...
        'Position', [120 10 350 20]);
    hAngle = uicontrol('Style', 'slider', 'Min', 1, 'Max', nAngles, ...
        'Value', 1, 'SliderStep', [1/(nAngles-1) 1/(nAngles-1)], ...
        'Position', [600 10 350 20]);

    hFrameLabel = uicontrol('Style', 'text', 'Position', [30 10 80 20], ...
        'String', 'Frame: 1', 'HorizontalAlignment', 'left');
    hAngleLabel = uicontrol('Style', 'text', 'Position', [960 10 80 20], ...
        'String', sprintf('Angle: %d°', anglesToSteer(1)), 'HorizontalAlignment', 'left');

    % Checkbox for each plot (bottom-left of each subplot)
    hAx1 = subplot(2,2,1); % Range-Doppler Map
    hCB1 = uicontrol('Style', 'checkbox', 'String', 'Log (dB)', ...
        'Position', [160 370 80 20], 'Value', 1, 'Parent', hFig); % Place it near subplot(2,2,1)
    hAx2 = subplot(2,2,2); % Range Profile
    hCB2 = uicontrol('Style', 'checkbox', 'String', 'Log (dB)', ...
        'Position', [690 370 80 20], 'Value', 1, 'Parent', hFig); % Place it near subplot(2,2,2)
    hAx3 = subplot(2,2,3); % Doppler Profile
    hCB3 = uicontrol('Style', 'checkbox', 'String', 'Log (dB)', ...
        'Position', [160 150 80 20], 'Value', 1, 'Parent', hFig); % Place it near subplot(2,2,3)
    hAx4 = subplot(2,2,4); % Range-Azimuth 3D

    % Callback function for updating plots
    function updatePlots(~,~)
        frameIdx = round(get(hFrame, 'Value'));
        angleIdx = round(get(hAngle, 'Value'));
        isLog1 = get(hCB1, 'Value'); % RD Map
        isLog2 = get(hCB2, 'Value'); % Range Profile
        isLog3 = get(hCB3, 'Value'); % Doppler Profile

        % Update labels
        set(hFrameLabel, 'String', sprintf('Frame: %d', frameIdx));
        set(hAngleLabel, 'String', sprintf('Angle: %d°', anglesToSteer(angleIdx)));

        to_plot = all_to_plot{frameIdx};
        to_plot = abs(to_plot);
        range_axis = all_range_axis{frameIdx};
        doppler_axis = all_doppler_axis{frameIdx};
        range_angle_stich = all_range_angle_stich{frameIdx};

        % Defensive dimension handling
        if ndims(to_plot) == 4
            rd_map_per_angle = to_plot(:,:,:,angleIdx); % (R, D, Rx)
            rd_map_per_angle = mean(rd_map_per_angle, 3); % (R, D)
        elseif ndims(to_plot) == 3
            if size(to_plot,3) >= angleIdx
                rd_map_per_angle = to_plot(:,:,angleIdx);
            else
                rd_map_per_angle = to_plot(:,:,1);
            end
        elseif ndims(to_plot) == 2
            rd_map_per_angle = to_plot;
        else
            error('Unexpected to_plot dimensions.');
        end

        % Range-Doppler Map (axes hAx1)
        axes(hAx1); cla(hAx1);
        if isLog1
            rd_disp = 20*log10(rd_map_per_angle + eps);
            imagesc(doppler_axis, range_axis, rd_disp);
            title('Range-Doppler Map (dB)');
            ylabel('Range (m)'); xlabel('Doppler (m/s)');
            colorbar;
        else
            imagesc(doppler_axis, range_axis, rd_map_per_angle);
            title('Range-Doppler Map (linear)');
            ylabel('Range (m)'); xlabel('Doppler (m/s)');
            colorbar;
        end
        axis xy;

        % Range Profile (axes hAx2) — plot each Doppler bin as a line
        axes(hAx2); cla(hAx2);
        if isLog2
            plot(range_axis, 20*log10(rd_map_per_angle + eps), 'LineWidth', 1.0);
            title('Range Profile (dB)');
            ylabel('Power (dB)');
        else
            plot(range_axis, rd_map_per_angle, 'LineWidth', 1.0);
            title('Range Profile (linear)');
            ylabel('Power (linear)');
        end
        xlabel('Range (m)');
        grid on;

        % Doppler Profile (axes hAx3) — plot each Range bin as a line
        axes(hAx3); cla(hAx3);
        if isLog3
            plot(doppler_axis, 20*log10(rd_map_per_angle' + eps), 'LineWidth', 1.0);
            title('Doppler Profile (dB)');
            ylabel('Power (dB)');
        else
            plot(doppler_axis, rd_map_per_angle', 'LineWidth', 1.0);
            title('Doppler Profile (linear)');
            ylabel('Power (linear)');
        end
        xlabel('Velocity (m/s)');
        grid on;

        % Range-Azimuth (stich map) with 3D perspective (view(0,60))
        axes(hAx4); cla(hAx4);
        numAngles = nAngles;
        indices_1D = 1:numel(range_axis);

        sine_theta = sind(anglesToSteer);
        cos_theta = sqrt(1-sine_theta.^2);
        [R_mat, sine_theta_mat] = meshgrid(range_axis(indices_1D), sine_theta);
        [~, cos_theta_mat] = meshgrid(range_axis(indices_1D), cos_theta);
        x_axis = R_mat.*cos_theta_mat;
        y_axis = R_mat.*sine_theta_mat;

        if ~ismatrix(range_angle_stich)
            range_angle_stich_2d = squeeze(range_angle_stich(:,:,1));
        else
            range_angle_stich_2d = range_angle_stich;
        end
        range_angle_stich_display = range_angle_stich_2d(indices_1D,:).';
        surf(y_axis, x_axis, abs(range_angle_stich_display).^0.2,'EdgeColor','none');
        view(0, 60);
        xlabel('meters')
        ylabel('meters')
        title('Stich range/azimuth (3D view)')
        colorbar;
        axis tight
    end

    % Set callbacks
    set(hFrame, 'Callback', @updatePlots);
    set(hAngle, 'Callback', @updatePlots);
    set(hCB1, 'Callback', @updatePlots);
    set(hCB2, 'Callback', @updatePlots);
    set(hCB3, 'Callback', @updatePlots);

    % Initial plot
    updatePlots();

end
