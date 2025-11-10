function interactive_txbf_viewer_gated()
% INTERACTIVE_TX_BF_VIEWER_PAGED
% Paged viewer for huge per-frame TXBF results.
% Adds an optional second window that shows DRONE-ONLY Doppler using a range gate.

    frames_per_batch = 200;
    data_folder  = './output/txbf_in_hov/';
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

    % Normalize each entry to have angle as 2nd dim (Nrange x Nangle, with Nangle>=1)
    for k = 1:numel(all_range_angle_stich)
        A = all_range_angle_stich{k};
        if isvector(A)
            % Make it Nrange x 1
            all_range_angle_stich{k} = A(:);
        elseif ndims(A) == 3
            % Your code later uses squeeze(..., :, :, 1); keep 2D Nrange x Nangle
            all_range_angle_stich{k} = squeeze(A(:,:,1));
            % else: already 2D => leave as is
        end
    end

    params_file = dir(fullfile(config_folder, '*_params.mat'));
    assert(~isempty(params_file), 'Cannot find *_params.mat in the given folder');
    params = load(fullfile(config_folder, params_file(1).name), 'params');
    params = params.params;
    anglesToSteer = params.anglesToSteer;
    nAngles = params.NumAnglesToSweep;

    % Batch state
    curr_batch_start = 1;
    curr_batch_end   = min(frames_per_batch, total_frames);
    batch_data = {};

    % ---- Drone Doppler window state (created on demand) ----
    droneWin.hFig2        = [];   % secondary figure
    droneWin.exists       = false;
    droneWin.auto_gate    = true;
    droneWin.gate_center_m = 80;   % initial guess (m)
    droneWin.gate_width_m  = 5;    % meters
    droneWin.v_exclude    = 0.5;   % m/s region around 0 to ignore for auto-detect

    % ---- Folding window state (created on demand) ----
    foldingWin.hFig3  = [];
    foldingWin.exists = false;
    foldingWin.jmin   = 2;   % folding search range, per paper
    foldingWin.jmax   = 20;
    foldingWin.fracOrder = 0.5; % FrFT order slider default
    foldingWin.logFrac = true; % Plot fractional spectrum in dB by default


    % ===================== UI: MAIN WINDOW =====================
    hFig = figure('Name', 'TX Beamforming Interactive Viewer (Gated)', ...
                  'NumberTitle', 'off', 'Position', [100 100 1200 800]);

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
    % If only one angle, lock the slider
    if nAngles == 1
        set(hAngle, 'Enable', 'off');   % or: set(hAngle,'Visible','off');
    end

    hFrameLabel = uicontrol('Style', 'text', 'Position', [470 20 100 20], ...
        'String', sprintf('Frame: %d/%d', curr_batch_start, total_frames), 'HorizontalAlignment', 'left');
    hAngleLabel = uicontrol('Style', 'text', 'Position', [1020 20 100 20], ...
        'String', sprintf('Angle: %d°', anglesToSteer(1)), 'HorizontalAlignment', 'left');

    % Add button to open second window
    hOpenDrone = uicontrol('Style','pushbutton','String','Drone Doppler…', ...
        'Units','normalized','Position',[0.32 0.04 0.12 0.04], ...
        'Callback',@openDroneWindow);
    uicontrol('Style','pushbutton','String','Folding / FrFT…', ...
        'Units','normalized','Position',[0.46 0.04 0.12 0.04], ...
        'Callback',@openFoldingWindow);

    % Axes + toggles
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

    % Load first batch and init
    load_current_batch();

    % -- Callbacks --
    set(hFrame, 'Callback', @updatePlots);
    set(hAngle, 'Callback', @updatePlots);
    set(hCB1, 'Callback', @updatePlots);
    set(hCB2, 'Callback', @updatePlots);
    set(hCB3, 'Callback', @updatePlots);

    % Initial draw
    updatePlots();

    % ===================== NESTED FUNCTIONS =====================

    function updatePlots(~,~)
        frameIdx = round(get(hFrame, 'Value'));
        % angleIdx = round(get(hAngle, 'Value'));
        angleIdx = min(max(1, round(get(hAngle,'Value'))), nAngles);

        isLog1 = get(hCB1, 'Value');
        isLog2 = get(hCB2, 'Value');
        isLog3 = get(hCB3, 'Value');

        globalFrameIdx = curr_batch_start + frameIdx - 1;
        set(hFrameLabel, 'String', sprintf('Frame: %d/%d', globalFrameIdx, total_frames));
        % set(hAngleLabel, 'String', sprintf('Angle: %d°', anglesToSteer(angleIdx)));
        set(hAngleLabel, 'String', sprintf('Angle: %d°', anglesToSteer(min(angleIdx, numel(anglesToSteer)))));


        % Load from batch_data
        D = batch_data{frameIdx};
        RD_map_abs_sq = (abs(D.RD_map)).^2; % [R D Ang]
        % to_plot = RD_map_abs_sq(:, :, angleIdx); % [R D]
        to_plot = angslice(RD_map_abs_sq, angleIdx);   % always [R x D]
        range_axis   = all_range_axis{globalFrameIdx};
        doppler_axis = all_doppler_axis{globalFrameIdx};

        % Limit range visually to 0..100 m
        max_range = 100; % meters
        idx_range = find(range_axis <= max_range);
        to_plot = to_plot(idx_range, :);
        range_axis = range_axis(idx_range);

        range_angle_stich = all_range_angle_stich{globalFrameIdx};

        % -------- RD Map --------
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

        % -------- Range Profile --------
        axes(hAx2); cla(hAx2);
        if isLog2
            plot(range_axis, 20*log10(max(mean(to_plot,2),2)+eps), 'LineWidth', 1.0);
            title('Range Profile (dB)', 'FontSize', 16);
            ylabel('Power (dB)', 'FontSize', 16);
        else
            plot(range_axis, mean(to_plot,2), 'LineWidth', 1.0);
            title('Range Profile (linear)', 'FontSize', 16);
            ylabel('Power (linear)', 'FontSize', 16);
        end
        xlabel('Range (m)', 'FontSize', 16);
        set(gca, 'FontSize', 14);

        % -------- Doppler Profile (ALL ranges, unchanged in main view) --------
        axes(hAx3); cla(hAx3);
        if isLog3
            plot(doppler_axis, 20*log10(max(mean(to_plot,1), 1)+eps), 'LineWidth', 1.0);
            title('Doppler Profile (All Ranges, dB)', 'FontSize', 16);
            ylabel('Power (dB)', 'FontSize', 16);
        else
            plot(doppler_axis, mean(to_plot,1), 'LineWidth', 1.0);
            title('Doppler Profile (All Ranges, linear)', 'FontSize', 16);
            ylabel('Power (linear)', 'FontSize', 16);
        end
        xlabel('Velocity (m/s)', 'FontSize', 16);
        set(gca, 'FontSize', 16);

        % -------- Range-Azimuth (stitch) --------
        axes(hAx4); cla(hAx4);
        % if ~ismatrix(range_angle_stich)
        %     range_angle_stich_2d = squeeze(range_angle_stich(:,:,1));
        % else
        %     range_angle_stich_2d = range_angle_stich;
        % end
        % indices_1D = 1:numel(range_axis);
        % sine_theta = sind(anglesToSteer);
        % cos_theta  = sqrt(1 - sine_theta.^2);
        % [R_mat, sine_theta_mat] = meshgrid(range_axis(indices_1D), sine_theta);
        % [~,  cos_theta_mat]     = meshgrid(range_axis(indices_1D), cos_theta);
        % x_axis = R_mat.*cos_theta_mat;
        % y_axis = R_mat.*sine_theta_mat;
        % range_angle_stich_2d = (range_angle_stich_2d(indices_1D,:).');
        % surf(y_axis, x_axis, abs(range_angle_stich_2d).^0.2,'EdgeColor','none');
        % view(0, 60);
        % xlabel('meters'); ylabel('meters');
        % title('Stich range/azimuth');
        % axis tight; colorbar;

        % 1) Normalize RA to 2-D [Nrange x Nangle]
        if ~ismatrix(range_angle_stich)
            range_angle_stich_2d = squeeze(range_angle_stich(:,:,1));
        else
            range_angle_stich_2d = range_angle_stich;
        end
        range_angle_stich_2d = range_angle_stich_2d(:,:);                 % ensure 2-D
        
        Nrange_RA = size(range_angle_stich_2d, 1);
        Nangle_RA = size(range_angle_stich_2d, 2);

        % 2) Align with axes you’re plotting
        % Clamp to the intersection so dimensions always agree
        Nrange_ax  = numel(range_axis);
        Nrange_use = min(Nrange_ax, Nrange_RA);
        range_angle_stich_2d = range_angle_stich_2d(1:Nrange_use, :);
        range_ax_use = range_axis(1:Nrange_use);

        % Make sure anglesToSteer matches the RA angle dimension
        anglesToSteer = anglesToSteer(:).';                     % row
        if numel(anglesToSteer) ~= Nangle_RA
            anglesToSteer = anglesToSteer(1:min(end, Nangle_RA));
        end
        Nangle_use = min(Nangle_RA, numel(anglesToSteer));
        range_angle_stich_2d = range_angle_stich_2d(:, 1:Nangle_use);
        angles_plot = anglesToSteer(1:Nangle_use);

        % 3) Build coordinates once dimensions are consistent
        sine_theta = sind(angles_plot);
        cos_theta  = sqrt(1 - sine_theta.^2);
        [R_mat,  sine_theta_mat]  = meshgrid(range_ax_use, sine_theta); % each is [Nangle_use x Nrange_use]
        [~,  cos_theta_mat]   = meshgrid(range_ax_use, cos_theta);
        X = R_mat .* cos_theta_mat;     % “x meters”
        Y = R_mat .* sine_theta_mat;    % “y meters”
        Z = abs(range_angle_stich_2d).^0.2;  % RA is [Nrange_use x Nangle_use]
        Z = Z.';   % -> [Nangle_use x Nrange_use] to match X,Y

        % 4) Plot: fallback for single-angle
        if Nangle_use > 1
            surf(Y, X, Z, 'EdgeColor','none');
            view(0, 60);
            xlabel('meters'); ylabel('meters');
            title('Stitch range/azimuth'); colorbar; axis tight;
        else
            % Single angle: a 2D image or a simple range profile is clearer than a “ribbon” surf
            imagesc(range_ax_use, 0, 20*log10(Z + eps));  % fake a 2D image row
            axis xy tight; colorbar;
            xlabel('Range (m)'); ylabel(sprintf('%d°', angles_plot(1)));
            title('Stitch (single angle)');
        end

        % Keep secondary window in sync, if open
        updateDroneWindow();
        updateFoldingWindow();
    end

    function load_current_batch()
        % Release previous batch data explicitly to save memory
        batch_data = cell(1, curr_batch_end - curr_batch_start + 1);
        for i = 1:(curr_batch_end - curr_batch_start + 1)
            frame_idx = curr_batch_start + i - 1;
            tmp = load(fullfile(frame_files(frame_idx).folder, frame_files(frame_idx).name));
            % ---- Normalize RD_map to always be [R x D x Ang] ----
            if isfield(tmp,'RD_map')
                if ndims(tmp.RD_map) == 2
                    % Single-angle data saved as [R x D]; make it [R x D x 1]
                    tmp.RD_map = reshape(tmp.RD_map, size(tmp.RD_map,1), size(tmp.RD_map,2), 1);
                end
            end
            batch_data{i} = tmp;
        end
        % Reset frame slider to new batch size
        batch_frames = curr_batch_end - curr_batch_start + 1;
        set(hFrame, 'Min', 1, 'Max', batch_frames, 'Value', 1, ...
            'SliderStep', [1/max(1,batch_frames-1), 1/max(1,batch_frames-1)]);
        updatePlots();
    end

    function next_batch(~,~)
        if curr_batch_end < total_frames
            curr_batch_start = curr_batch_start + frames_per_batch;
            curr_batch_end   = min(curr_batch_start + frames_per_batch - 1, total_frames);
            load_current_batch();
        end
    end

    function prev_batch(~,~)
        if curr_batch_start > 1
            curr_batch_start = max(1, curr_batch_start - frames_per_batch);
            curr_batch_end   = min(curr_batch_start + frames_per_batch - 1, total_frames);
            load_current_batch();
        end
    end

    % ===================== SECOND WINDOW =====================

    function openDroneWindow(~,~)
        if droneWin.exists && isvalid(droneWin.hFig2)
            figure(droneWin.hFig2);
            if ~foldingWin.exists || ~isvalid(foldingWin.hFig3)
                openFoldingWindow();
            end 
            return;
        end

        droneWin.hFig2 = figure('Name','Drone-Only Doppler','NumberTitle','off', ...
                                'Position',[150 150 900 600]);

        t = tiledlayout(droneWin.hFig2,2,2,'Padding','compact','TileSpacing','compact');
        droneWin.axRD    = nexttile(t,1);     % RD
        droneWin.axRange = nexttile(t,2);     % Range
        droneWin.axDopp  = nexttile(t,[1 2]); % Doppler (wide)

        % Bottom controls
        uicontrol(droneWin.hFig2,'Style','checkbox','String','Auto gate','Value',1, ...
            'Units','normalized','Position',[0.02 0.01 0.1 0.05], ...
            'Callback',@(s,~) setAutoGate2(s));
        uicontrol(droneWin.hFig2,'Style','text','String','Center(m):', ...
            'Units','normalized','Position',[0.14 0.01 0.08 0.04],'HorizontalAlignment','right');
        droneWin.edCenter = uicontrol(droneWin.hFig2,'Style','edit','String',num2str(droneWin.gate_center_m), ...
            'Units','normalized','Position',[0.23 0.012 0.06 0.05], ...
            'Callback',@(s,~) setGateParams2());
        uicontrol(droneWin.hFig2,'Style','text','String','Width(m):', ...
            'Units','normalized','Position',[0.31 0.01 0.08 0.04],'HorizontalAlignment','right');
        droneWin.edWidth  = uicontrol(droneWin.hFig2,'Style','edit','String',num2str(droneWin.gate_width_m), ...
            'Units','normalized','Position',[0.40 0.012 0.06 0.05], ...
            'Callback',@(s,~) setGateParams2());
        droneWin.btPick   = uicontrol(droneWin.hFig2,'Style','pushbutton','String','Pick gate on Range', ...
            'Units','normalized','Position',[0.50 0.012 0.15 0.05], ...
            'Callback',@pickGateFromRange2);

        droneWin.txtInfo  = uicontrol(droneWin.hFig2,'Style','text','String','', ...
            'Units','normalized','Position',[0.70 0.012 0.28 0.05],'HorizontalAlignment','left');

        droneWin.exists = true;
        updateDroneWindow();
        % Also open the Folding window (separate figure, no changes to other windows)
        openFoldingWindow();

        % Secondary window control callbacks
        function setAutoGate2(src)
            droneWin.auto_gate = logical(get(src,'Value'));
            updateDroneWindow();
        end
        function setGateParams2()
            droneWin.gate_center_m = str2double(get(droneWin.edCenter,'String'));
            droneWin.gate_width_m  = str2double(get(droneWin.edWidth,'String'));
            updateDroneWindow();
        end
        function pickGateFromRange2(~,~)
            if ~isvalid(droneWin.hFig2), return; end
            figure(droneWin.hFig2); axes(droneWin.axRange);
            [x,~] = ginput(1);
            if ~isempty(x)
                droneWin.gate_center_m = x;
                set(droneWin.edCenter,'String',sprintf('%.2f',x));
                droneWin.auto_gate = false;
                updateDroneWindow();
            end
        end
    end

    function updateDroneWindow()
        % No-op if window not open
        if ~droneWin.exists || ~isvalid(droneWin.hFig2)
            return;
        end

        % Sync with current main selection and log toggles
        frameIdx = round(get(hFrame, 'Value'));
        % angleIdx = round(get(hAngle, 'Value'));
        angleIdx = min(max(1, round(get(hAngle,'Value'))), nAngles);
        globalFrameIdx = curr_batch_start + frameIdx - 1;
        isLog1 = get(hCB1, 'Value'); % for RD
        isLog2 = get(hCB2, 'Value'); % for Range
        isLog3 = get(hCB3, 'Value'); % for Doppler

        D = batch_data{frameIdx};
        RD_map_abs = abs(D.RD_map).^2;        % [R D Ang]
        % to_plot = RD_map_abs(:,:,angleIdx);   % [R D]
        to_plot = angslice(RD_map_abs, angleIdx);   % -> [R x D]
        range_axis   = all_range_axis{globalFrameIdx};
        doppler_axis = all_doppler_axis{globalFrameIdx};

        % Limit range visually
        max_range = 40; 
        idx_range = find(range_axis <= max_range);
        to_plot = to_plot(idx_range, :);
        range_axis = range_axis(idx_range);

        % Compute gate
        [gate_idx, gate_center, gate_width] = compute_gate_for_drone( ...
            to_plot, range_axis, doppler_axis, droneWin.auto_gate, ...
            droneWin.gate_center_m, droneWin.gate_width_m, droneWin.v_exclude);

        % Keep UI synced
        droneWin.gate_center_m = gate_center;
        droneWin.gate_width_m  = gate_width;
        if isvalid(droneWin.edCenter), set(droneWin.edCenter,'String',sprintf('%.2f',gate_center)); end
        if isvalid(droneWin.edWidth),  set(droneWin.edWidth, 'String',sprintf('%.2f',gate_width));  end

        % ----- RD map with gate band -----
        axes(droneWin.axRD); cla(droneWin.axRD);
        if isLog1
            imagesc(doppler_axis, range_axis, 20*log10(to_plot+eps)); axis xy;
            title('RD (dB) with Gate');
        else
            imagesc(doppler_axis, range_axis, to_plot); axis xy;
            title('RD (linear) with Gate');
        end
        xlabel('Doppler (m/s)'); ylabel('Range (m)');
        hold on;
        half_w = gate_width/2;
        patch([min(doppler_axis) max(doppler_axis) max(doppler_axis) min(doppler_axis)], ...
              [gate_center-half_w gate_center-half_w gate_center+half_w gate_center+half_w], ...
              'w','FaceAlpha',0.08,'EdgeColor','w','LineStyle','--');
        hold off; colorbar;

        % ----- Folding per range bin (per paper) -----
        % to_plot is [R x D] power map for current angle; use linear power
        % jmin = 2; jmax = 20;   % same empirical range used in the paper
        % [P_fold, j_best] = compute_range_folding_values(to_plot, jmin, jmax);  % P per range

        % ----- Range profile with gate overlay -----
        axes(droneWin.axRange); cla(droneWin.axRange);
        rp = mean(to_plot,2);
        if isLog2
            plot(range_axis, 20*log10(max(rp,2)+eps),'LineWidth',1.0); hold on;
            plot(range_axis(gate_idx),20*log10(max(rp(gate_idx),2)+eps),'LineWidth',2.0);
            title('Range Profile (dB)'); ylabel('Power (dB)');
        else
            plot(range_axis, rp,'LineWidth',1.0); hold on;
            plot(range_axis(gate_idx), rp(gate_idx),'LineWidth',2.0);
            title('Range Profile (linear)'); ylabel('Power (linear)');
        end
        xlabel('Range (m)');
        yl = ylim;
        plot([gate_center-half_w gate_center-half_w], yl, '--');
        plot([gate_center+half_w gate_center+half_w], yl, '--');
        hold off; grid on;

        % ===== Add TOP X-AXIS LABELS with folding values (no new plot) =====
        jmin = 2; jmax = 20; % choose per your Doppler length / rotor cadence range
        [P_fold, ~] = compute_range_folding_values(to_plot, jmin, jmax);
        % label_folding_topaxis(droneWin.axRange, range_axis, P_fold, '%.1f');

        % (Optional) Show a quick comparison in the info label:
        P_gate_mean = mean(P_fold(gate_idx));
        P_oth_mean  = mean(P_fold(setdiff(1:numel(range_axis), gate_idx)));

        % ----- Drone-only Doppler (gated rows only) -----
        axes(droneWin.axDopp); cla(droneWin.axDopp);
        doppler_gated = mean(to_plot(gate_idx,:),1);
        if isLog3
            plot(doppler_axis, 20*log10(max(doppler_gated,1)+eps), 'LineWidth',1.5);
            title('Drone-Only Doppler (dB)'); ylabel('Power (dB)');
        else
            plot(doppler_axis, doppler_gated, 'LineWidth',1.5);
            title('Drone-Only Doppler (linear)'); ylabel('Power (linear)');
        end
        xlabel('Velocity (m/s)'); grid on;

        set(droneWin.txtInfo,'String', ...
            sprintf('Frame %d | Angle %d° | Gate %.2fm±%.2fm | Fold(gate)=%.1f, Fold(else)=%.1f', ...
            globalFrameIdx, anglesToSteer(angleIdx), gate_center, gate_width, P_gate_mean, P_oth_mean));
        updateFoldingWindow();
    end

    function openFoldingWindow(~,~)
        if foldingWin.exists && isvalid(foldingWin.hFig3)
            figure(foldingWin.hFig3); % focus
            return;
        end

        % Create a slim, dedicated figure for folding values only
        % foldingWin.hFig3 = figure('Name','Folding Values (per range)', ...
        %                           'NumberTitle','off', 'Position',[220 220 900 260]);

        % Create a dedicated figure that will host folding, FrFT spectrum and controls
        foldingWin.hFig3 = figure('Name','Folding & Fractional Spectrum', ...
            'NumberTitle','off', 'Position',[220 220 950 620], ...
            'CloseRequestFcn',@closeFoldingWindow);

        % One axis that fills most of the figure
        % foldingWin.ax = axes('Parent', foldingWin.hFig3, 'Position',[0.08 0.22 0.88 0.72]);
        tFold = tiledlayout(foldingWin.hFig3,2,1,'Padding','compact','TileSpacing','compact');
        foldingWin.ax = nexttile(tFold,1);
        foldingWin.axFrft = nexttile(tFold,2);

        % Optional controls to tweak jmin/jmax
        uicontrol(foldingWin.hFig3,'Style','text','Units','normalized','Position',[0.08 0.02 0.05 0.05],...
            'String','j_{min}','HorizontalAlignment','right');
        foldingWin.edJmin = uicontrol(foldingWin.hFig3,'Style','edit','Units','normalized','Position',[0.14 0.02 0.06 0.05],...
            'String',num2str(foldingWin.jmin),'Callback',@(s,~) setFoldParams());
        uicontrol(foldingWin.hFig3,'Style','text','Units','normalized','Position',[0.22 0.02 0.05 0.05],...
            'String','j_{max}','HorizontalAlignment','right');
        foldingWin.edJmax = uicontrol(foldingWin.hFig3,'Style','edit','Units','normalized','Position',[0.28 0.02 0.06 0.05],...
            'String',num2str(foldingWin.jmax),'Callback',@(s,~) setFoldParams());

        % FrFT slider and label
        uicontrol(foldingWin.hFig3,'Style','text','Units','normalized','Position',[0.40 0.02 0.10 0.05],...
            'String','FrFT order','HorizontalAlignment','right');
        foldingWin.slFrft = uicontrol(foldingWin.hFig3,'Style','slider','Units','normalized',...
            'Position',[0.51 0.025 0.25 0.04],'Min',0,'Max',1,'Value',foldingWin.fracOrder,...
            'SliderStep',[0.1 0.1],'Callback',@(s,~) setFracOrder(s));
        foldingWin.txtFrft = uicontrol(foldingWin.hFig3,'Style','text','Units','normalized','Position',[0.78 0.02 0.15 0.05],...
            'String',sprintf('Order: %.1f',foldingWin.fracOrder),'HorizontalAlignment','left');

        foldingWin.exists = true;
        updateFoldingWindow();

        function setFoldParams()
            foldingWin.jmin = max(2, round(str2double(get(foldingWin.edJmin,'String'))));
            foldingWin.jmax = max(foldingWin.jmin, round(str2double(get(foldingWin.edJmax,'String'))));
            set(foldingWin.edJmin,'String',num2str(foldingWin.jmin));
            set(foldingWin.edJmax,'String',num2str(foldingWin.jmax));
            updateFoldingWindow();
        end

        function setFracOrder(src)
            val = get(src,'Value');
            val = round(val*10)/10; % snap to 0.1 steps
            foldingWin.fracOrder = max(0, min(1, val));
            if isvalid(foldingWin.slFrft)
                set(foldingWin.slFrft,'Value',foldingWin.fracOrder);
            end
            if isfield(foldingWin,'txtFrft') && isvalid(foldingWin.txtFrft)
                set(foldingWin.txtFrft,'String',sprintf('Order: %.1f',foldingWin.fracOrder));
            end
            updateFoldingWindow();
        end

        function closeFoldingWindow(src, ~)
            foldingWin.exists = false;
            delete(src);
        end
    end

    function updateFoldingWindow()
        if ~foldingWin.exists || ~isvalid(foldingWin.hFig3)
            return;
        end

        % Pull the same frame/angle selection as the other windows
        frameIdx = round(get(hFrame, 'Value'));
        % angleIdx = round(get(hAngle, 'Value'));
        angleIdx = min(max(1, round(get(hAngle,'Value'))), nAngles);
        globalFrameIdx = curr_batch_start + frameIdx - 1;

        % Assemble the same to_plot & axes (linear power)
        D = batch_data{frameIdx};
        RD_map_slice_complex = D.RD_map(:,:,angleIdx); % retain complex data
        RD_map_abs = abs(D.RD_map).^2;      % [R D Ang]
        % to_plot = RD_map_abs(:,:,angleIdx); % [R D]
        to_plot = angslice(RD_map_abs, angleIdx);   % -> [R x D]
        range_axis   = all_range_axis{globalFrameIdx};
        doppler_axis = all_doppler_axis{globalFrameIdx};

        % Keep the same range trimming as the other windows
        max_range = 100;
        idx_range = find(range_axis <= max_range);
        to_plot = to_plot(idx_range, :);
        RD_map_slice_complex = RD_map_slice_complex(idx_range, :);
        range_axis = range_axis(idx_range);

        % Compute/refresh the *current* gate (only to draw vertical lines here)
        [gate_idx, gate_center, gate_width] = compute_gate_for_drone( ...
            to_plot, range_axis, doppler_axis, droneWin.auto_gate, ...
            droneWin.gate_center_m, droneWin.gate_width_m, droneWin.v_exclude);

        % Folding values per range bin (paper definition)
        [P_fold, ~] = compute_range_folding_values(to_plot, foldingWin.jmin, foldingWin.jmax);

        % Draw folding values per range bin
        axes(foldingWin.ax); cla(foldingWin.ax);
        plot(range_axis, P_fold, 'LineWidth',1.3); grid on;
        xlabel('Range (m)');
        ylabel('Folding value');
        title(sprintf('Folding values per range (j = %d..%d)', foldingWin.jmin, foldingWin.jmax));

        % Overlay the current gate as two dashed verticals
        yl = ylim; half_w = gate_width/2; hold on;
        plot([gate_center-half_w gate_center-half_w], yl, '--');
        plot([gate_center+half_w gate_center+half_w], yl, '--');
        hold off;

        % Fractional Fourier spectrum of gated Doppler profile
        axes(foldingWin.axFrft); cla(foldingWin.axFrft);
        if isfield(foldingWin,'txtFrft') && isvalid(foldingWin.txtFrft)
            set(foldingWin.txtFrft,'String',sprintf('Order: %.1f',foldingWin.fracOrder));
        end
        if isfield(foldingWin,'cbFrftLog') && isvalid(foldingWin.cbFrftLog)
            set(foldingWin.cbFrftLog,'Value',foldingWin.logFrac);
        end
        doppler_gated_complex = mean(RD_map_slice_complex(gate_idx,:),1);
        doppler_power = abs(doppler_gated_complex).^2;
        if isempty(doppler_gated_complex)
            frft_spec = zeros(size(doppler_axis));
            doppler_power = zeros(size(doppler_axis));
        else
            frft_spec = abs(frft(doppler_gated_complex(:), foldingWin.fracOrder)).^2;
            frft_spec = frft_spec(:).';
            if numel(frft_spec) ~= numel(doppler_axis)
                frft_spec = interp1(linspace(doppler_axis(1), doppler_axis(end), numel(frft_spec)), frft_spec, doppler_axis, 'linear', 'extrap');
            end
        end

        if foldingWin.logFrac
            doppler_plot = 10*log10(max(doppler_power, eps));
            frft_plot = 10*log10(max(frft_spec, eps));
            yLabelStr = 'Power (dB)';
            titleStr = 'Doppler vs Fractional Fourier Spectrum (dB)';
        else
            doppler_plot = doppler_power;
            frft_plot = frft_spec;
            yLabelStr = 'Power (linear)';
            titleStr = 'Doppler vs Fractional Fourier Spectrum (linear)';
        end

        plot(doppler_axis, doppler_plot, 'LineWidth',1.3, 'DisplayName','Existing Doppler spectrum'); hold on;
        plot(doppler_axis, frft_plot, 'LineWidth',1.3, 'DisplayName',sprintf('FrFT spectrum (order %.1f)', foldingWin.fracOrder));
        hold off;
        legend('Location','best');
        grid on;
        xlabel('Velocity (m/s)');
        ylabel(yLabelStr);
        title(titleStr);
    end

    function X = angslice(M, idx)
        % Returns the [R x D] slice at angle idx, tolerating both 2D and 3D
        if ndims(M) == 2
            X = M;                       % already [R x D] for single angle
        else
            idx = min(max(1, idx), size(M,3));
            X = M(:,:,idx);
        end
    end

end

function [gate_idx, gate_center_m, gate_width_m] = compute_gate_for_drone( ...
        to_plot, range_axis, doppler_axis, auto_gate, gate_center_m, gate_width_m, v_exclude)
    % Auto-gate on strongest non-zero-Doppler range; else use manual center/width.
    if auto_gate
        nz = abs(doppler_axis) > v_exclude;
        rp_nz = mean(to_plot(:, nz), 2);
        if ~any(nz)
            rp_nz = mean(to_plot,2);
        end
        [~, pk] = max(rp_nz);
        gate_center_m = range_axis(pk);
    end
    % Ensure >= 1 bin in gate
    if numel(range_axis) > 1
        dr = mean(diff(range_axis));
    else
        dr = 1;
    end
    half_w = max(gate_width_m/2, 0.5*dr);
    mask = (range_axis >= gate_center_m - half_w) & (range_axis <= gate_center_m + half_w);
    if ~any(mask)
        [~, nn] = min(abs(range_axis - gate_center_m));
        mask(nn) = true;
    end
    gate_idx = find(mask);
end

% function label_folding_topaxis(axRange, range_axis, P, fmt)
% % Overlay ONLY a top x-axis with tick labels showing folding values.
% % - axRange: handle of your Range Profile axis
% % - range_axis: vector of x positions (same length/order as P)
% % - P: folding value per range bin (from compute_range_folding_values)
% % - fmt: optional sprintf format for labels (e.g., '%.1f')
%     if nargin < 4, fmt = '%.1f'; end
% 
%     % Remove any previous overlay for a clean refresh
%     old = findobj(get(axRange,'Parent'), 'Type','axes', 'Tag','FoldingTopAxisLabels');
%     if ~isempty(old), delete(old); end
% 
%     % Use bottom axis ticks to avoid clutter
%     xt  = get(axRange, 'XTick');
%     xlm = get(axRange, 'XLim');
% 
%     % Interpolate folding values at those tick positions
%     % Use 'nearest' so labels correspond to actual bins
%     Pt = interp1(range_axis(:), P(:), xt(:), 'nearest', 'extrap');
% 
%     % Create a transparent axis on top to host the labels (no data plotted)
%     basePos = get(axRange, 'Position');
%     axTop = axes('Position', basePos, ...
%                  'Color','none', 'XAxisLocation','top', ...
%                  'YAxisLocation','right', 'YColor','none', ...
%                  'XLim', xlm, 'Tag','FoldingTopAxisLabels', ...
%                  'HitTest','off', 'PickableParts','none', ...
%                  'Box','off');
% 
%     set(axTop, 'XTick', xt);
%     set(axTop, 'XTickLabel', arrayfun(@(v) sprintf(fmt, v), Pt, 'UniformOutput', false));
%     xlabel(axTop, 'Folding value'); % top-axis label
% end

function draw_folding_topaxis(axRange, range_axis, P)
% DRAW_FOLDING_TOPAXIS
% Overlays a slim axis above axRange (your Range Profile) that plots
% the folding value P(range). Shares x-limits; top x-axis shows range; label top.
    % Remove any previous overlay
    old = findobj(get(axRange,'Parent'),'Type','axes','Tag','FoldingTopAxis');
    if ~isempty(old), delete(old); end

    % Positioning: create a shallow axis above the Range Profile axis
    basePos = get(axRange,'Position');
    ht   = basePos(4);   % height
    gap  = 0.02;         % small gap
    frac = 0.22;         % top strip height fraction relative to base axis
    topPos = [basePos(1), basePos(2)+ht+gap, basePos(3), ht*frac];

    axTop = axes('Position', topPos, 'Tag','FoldingTopAxis');
    plot(range_axis, P, 'LineWidth', 1.2); grid on;
    xlim(axTop, xlim(axRange));  % lock x-range
    set(axTop, 'XAxisLocation','top', 'YAxisLocation','right');
    set(axTop, 'XTickLabel', []);          % keep the main axis as the place for range labels
    ylabel(axTop, 'Folding');              % shows folding value scale
    % Optional: make background transparent-ish over figure
    set(axTop, 'Color', 'none'); 
end

