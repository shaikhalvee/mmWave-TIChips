function interactive_pmmmap_viewer()
% INTERACTIVE_PMMMAP_VIEWER_PAGED: Interactive viewer for PMMmap [R Rx Ang F]

    % Load PMMmap: [R Rx Ang F]
data_folder = './output/txbf_13_7_25/';
S = load(fullfile(data_folder, 'PMMmap.mat'), 'PMMmap');
PMMmap = S.PMMmap;
[numRangeBins, ~, ~, numFrames] = size(PMMmap);

% Combine across Rx (max)
PMMmap_rx = squeeze(max(PMMmap, [], 2));    % [R Ang F]

% Combine across Angles (mean)
PMMmap_rf = squeeze(max(PMMmap_rx, [], 2));    % [R F]

% (Optional: real range axis, or just bin index)
try
    params_file = dir(fullfile(data_folder, '*_params.mat'));
    params = load(fullfile(data_folder, params_file(1).name), 'params');
    params = params.params;
    c = physconst('LightSpeed');
    fs = params.Sampling_Rate_ksps * 1e3;
    slope = params.Slope_MHzperus * 1e12;
    N = numRangeBins;
    range_axis = (0:N-1) * c * fs / (2 * slope * N); % meters
catch
    range_axis = 1:numRangeBins;
end

% Show Range–Frame heatmap
figure;
imagesc(1:numFrames, range_axis, PMMmap_rf); 
axis xy;
xlabel('Frame');
ylabel('Range (m)');
title('Range–Frame PMMmap)');
colorbar;
set(gca,'FontSize',16);

figure('Name', 'Range Profile per Frame');
hAx = axes;
hSlider = uicontrol('Style', 'slider', ...
    'Min', 1, 'Max', numFrames, 'Value', 1, ...
    'SliderStep', [1/(numFrames-1), 10/(numFrames-1)], ...
    'Position', [100 20 400 20], ...
    'Callback', @updateProfile);
hTxt = uicontrol('Style','text','Position',[520 20 80 20],'String','Frame: 1');

    function updateProfile(~,~)
        frameIdx = round(get(hSlider,'Value'));
        set(hTxt, 'String', sprintf('Frame: %d', frameIdx));
        plot(hAx, range_axis, PMMmap_rf(:,frameIdx), 'LineWidth', 1.5);
        xlabel(hAx,'Range (m)'); ylabel(hAx,'PMM score');
        title(hAx, sprintf('Range Profile (Frame %d)', frameIdx));
        set(hAx,'FontSize',16);
    end

updateProfile();


end
