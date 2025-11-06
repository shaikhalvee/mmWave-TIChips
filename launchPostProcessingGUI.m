%% File: launchPostProcessingGUI.m
function launchPostProcessingGUI(testRoot)
    % Interactive RD FFT viewer with RX dropdown & frame slider
    outDir = fullfile('output', testRoot);
    S = load(fullfile(outDir,'rangeDopplerMap.mat'), 'rangeDopplerFFTData','mmWaveDevice');
    cube       = abs(S.rangeDopplerFFTData);
    mmWaveDevice = S.mmWaveDevice;
    res        = mmWaveDevice.range_res;
    vel        = -mmWaveDevice.v_max : mmWaveDevice.v_res : (mmWaveDevice.v_max - mmWaveDevice.v_res);
    [R,~,X,F]  = size(cube);

    % Build masked ranges
    ranges = (0:R-1)*res;

    % GUI elements
    fig = figure('Name','RD FFT Viewer','NumberTitle','off');
    ax  = axes('Parent',fig,'Position',[0.1 0.3 0.85 0.65]);
    % RX dropdown
    uicontrol('Style','popupmenu','String',arrayfun(@(x)sprintf('RX %d',x),1:X,'Uni',false),...
              'Units','normalized','Position',[0.05 0.15 0.2 0.05],...
              'Callback',@updatePlot,'Tag','rxMenu');
    % Frame slider
    uicontrol('Style','slider','Min',1,'Max',F,'Value',1, ...
              'Units','normalized','Position',[0.3 0.1 0.5 0.05],...
              'SliderStep',[1/(F-1) 10/(F-1)], 'Callback',@updatePlot,'Tag','frameSlider');
    % Frame label
    txt = uicontrol('Style','text','Units','normalized','Position',[0.82 0.1 0.12 0.05],'String','Frame:1');

    updatePlot();

    function updatePlot(~,~)
        rxVal = findobj('Tag','rxMenu'); ch = rxVal.Value;
        slVal = findobj('Tag','frameSlider'); frm = round(slVal.Value);
        txt.String = sprintf('Frame:%d',frm);
        img = squeeze(cube(:,:,ch,frm));
        imagesc(ax,vel,ranges,img);
        set(ax,'YDir','normal');
        xlabel(ax,'Velocity (m/s)'); ylabel(ax,'Range (m)');
        title(ax,sprintf('RD FFT — RX%d, Frame%d',ch,frm)); colorbar(ax);
    end
end
