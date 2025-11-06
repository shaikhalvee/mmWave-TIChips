%% File: mmHawkeyeMain.m

clearvars;

% Entry point for unified pipeline
dataPath = "D:\Documents\Drone_Data\single_chip\pm_28_4_25";   % Setup JSON (includes bin list & config)

setupFilePattern = fullfile(dataPath, '*.setup.json');

setupFile = dir(setupFilePattern);
setupFile = fullfile(setupFile(1).folder, setupFile(1).name);

[testRootFolder, ~, ~] = fileparts(setupFile);
[~, testRoot] = fileparts(testRootFolder);

% Outputs
saveRaw = 'rawData.mat';
save1D  = 'rangeFFT.mat';
save2D  = 'rangeDopplerMap.mat';

% Instantiate device & pipeline
device   = iwr6843mmWaveDevice(setupFile);
pipeline = ProcessingPipeline(device);

% Export data
pipeline.exportAll(saveRaw, save1D, save2D);

% Launch GUI
launchPostProcessingGUI(pipeline);
% rdFFTViewer(testRoot)


% Cleanup
device.closeFiles();
