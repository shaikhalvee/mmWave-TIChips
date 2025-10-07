% get single chip adc data from /output. 
% visualize per Rx per Tx data

function rdFFTViewer(testRoot)
    % Load the precomputed RD FFT cube
    outDir = fullfile('output', testRoot);
    S = load(fullfile(outDir,'rangeDopplerMap.mat'), 'rangeDopplerFFTData','mmWaveDevice');
    % radar_cube = S.fft_complex_radar_cube;
    
    radar_cube = abs(S. rangeDopplerFFTData);
    mmWaveDevice = S.mmWaveDevice;

    ranges = (1:mmWaveDevice.num_adc_sample_per_chirp-1) * mmWaveDevice.range_res;
    velocities = -mmWaveDevice.v_max : mmWaveDevice.v_res : (mmWaveDevice.v_max - mmWaveDevice.v_res);
    range_limit = 20;  % meters
    range_idx = ranges <= range_limit;
    limited_ranges = ranges(range_idx);

    % limited_ranges = S.limited_ranges;
    % range_idx = S.range_idx;
    [~, ~, numRx, numFrames] = size(radar_cube); % [numRangeBins, numDopplerBins, numFrames, numRx]

    % Create figure and axes
    hFig = figure('Name','Range–Doppler FFT Viewer','NumberTitle','off');
    hAx  = axes('Parent',hFig, 'Position', [0.1 0.25 0.85 0.7]);

    % Dropdown (popup) menu for RX channel selection
    rxList = arrayfun(@(x)sprintf('RX %d',x), 1:numRx, 'UniformOutput',false);
    hPopup = uicontrol( ...
        'Style','popupmenu', ...
        'String',rxList, ...
        'Value',1, ...
        'Units','normalized', ...
        'Position',[0.05 0.05 0.2 0.05], ...
        'Callback',@updatePlot);

    % Slider for frame selection
    hSlider = uicontrol( ...
        'Style','slider', ...
        'Min',1, ...
        'Max',numFrames, ...
        'Value',1, ...
        'SliderStep',[1/(numFrames-1) , 10/(numFrames-1)], ...
        'Units','normalized', ...
        'Position',[0.3 0.05 0.5 0.05], ...
        'Callback',@updatePlot);

    % Text label to show current frame number
    hTxt = uicontrol( ...
        'Style','text', ...
        'Units','normalized', ...
        'Position',[0.82 0.05 0.12 0.05], ...
        'String','Frame: 1');

    % Initial plot
    updatePlot();

    function updatePlot(~,~)
        % Read UI values
        rx = get(hPopup,'Value');
        frame = round(get(hSlider,'Value'));
        set(hTxt,'String',sprintf('Frame: %d',frame));

        % Extract the RD FFT slice for (range × doppler)
        rdSlice = radar_cube(range_idx,:, rx, frame);
        % rdSlice = 20*log10(radar_cube(range_idx,:, rx, frame));
        % frame_data = 20*log10(norm_fft(range_idx,:));

        % Display
        imagesc(hAx, velocities, limited_ranges, rdSlice);
        set(hAx,'YDir','normal');
        xlabel(hAx,'Doppler Bin');
        ylabel(hAx,'Range Bin');
        title(hAx,sprintf('RD FFT — RX %d, Frame %d', rx, frame));
        colorbar('peer',hAx);
    end
end
