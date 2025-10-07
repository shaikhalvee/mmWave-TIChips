clearvars;

% load variables
testFolder = 'drn_25_5_2025_70m_2';

outDir = fullfile('output', testFolder);
S = load(fullfile(outDir,'PMMmap.mat'), 'PMMmap');
P = load(fullfile(outDir,'rangeDopplerMap.mat'),'mmWaveDevice');

PMMmap = S.PMMmap;
mmWaveDevice = P.mmWaveDevice;
numRangeBins = mmWaveDevice.num_adc_sample_per_chirp;
numDopplerBins = mmWaveDevice.num_chirp_per_frame;
numRx = mmWaveDevice.num_rx_chnl;
numFrames = mmWaveDevice.total_frames;

%--- combine across Rx channels -------------
% e.g. take maximum across Rx
PMM_combined = max(PMMmap, [], 3);   % size = [R × F]
% visualize the Range–Time PMM heat‐map
imagesc((1:numFrames), (1:numRangeBins), PMM_combined(1:numRangeBins,:));
axis xy;
xlabel('Frame index');
ylabel('Range bin');
title('Range–Time PMM folding map', testFolder);
colorbar;

