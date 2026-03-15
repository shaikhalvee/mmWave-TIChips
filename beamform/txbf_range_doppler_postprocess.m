%----------------------------------------------------------------------
% TX Beamforming Range-Doppler Processing (single angle, long-range)
%----------------------------------------------------------------------
close all; clc; clearvars;


% ----------------- USER CONFIG ---------------------------------------
adc_data_folder = 'G:\RADAR_DATA\in_txbf_hand_test';
[~, testRootFolder, ~] = fileparts(adc_data_folder);
output_folder =  ['./output/' testRootFolder]; 
oldParamsFile = [output_folder filesep testRootFolder '_params.mat'];
frame_folder = [output_folder filesep 'rangeDopplerFFTmap_10_rx_mn/'];
calib_file = './input/calibrConfig/calibrateResults_dummy.mat';

% ----------------- LOAD PARAMS FROM JSON & CALIB ---------------------
oldParams = load(oldParamsFile, 'params');
params = oldParams.params;
 
configFromAdcData = get_radar_config(params.anglesToSteer, [adc_data_folder '\']);
params.Slope_MHzperus = configFromAdcData.Slope_MHzperus;
params.TI_Cascade_RX_ID = configFromAdcData.TI_Cascade_RX_ID;
params.numRX = configFromAdcData.numRX;
params.D_RX = configFromAdcData.D_RX;
params.calibrationInterp = configFromAdcData.calibrationInterp;
params.phaseCalibOnly = configFromAdcData.phaseCalibOnly;

numAdc = params.Samples_per_Chirp;
numChirp = params.nchirp_loops;
numRx = params.numRX;
numAngle = params.NumAnglesToSweep;

% clutter & noise handle
dcOffsetRemoval = 1;
dopplerClutterRemoval = 0;

% Calibration (RX phase)
load(calib_file, 'calibResult');
BF_MIMO_ref = calibResult.RxMismatch;

% --------------- LOAD FILES & PROCESS EACH FRAME ----------------------
[fileIdx_unique] = getUniqueFileIdx(adc_data_folder);

% all_RD_map = {};        % cell array for all frames (if #frames can differ per file)
all_range_axis = {};
all_doppler_axis = {};
all_range_angle_stich = {};
all_to_plot = {};

frameCounter = 2;

for i_file = 1:numel(fileIdx_unique)
    [fileNameStruct] = getBinFileNames_withIdx(adc_data_folder, fileIdx_unique{i_file});
    [numValidFrames, ~] = getValidNumFrames(fullfile(adc_data_folder, fileNameStruct.masterIdxFile));

    for frameId = 2:numValidFrames
        params.frameId = frameId;

        % ----------------- LOAD RAW ADC DATA (ALL RX, 1 ANGLE) ----------
        radar_data_txbf = fcn_read_AdvFrmConfig_BF_Json(fileNameStruct, params);
        radar_data_txbf = radar_data_txbf(:,:,params.TI_Cascade_RX_ID,:);

        % --------------- RANGE-DOPPLER PROCESSING & PLOT ---------------
        [RD_map, range_axis, doppler_axis, params] = calc_range_doppler_bmfrm( ...
                    radar_data_txbf, params, BF_MIMO_ref, dcOffsetRemoval, dopplerClutterRemoval);

        % Store for saving later
        % all_RD_map{end+1} = RD_map;
        all_range_axis{end+1} = range_axis;
        all_doppler_axis{end+1} = doppler_axis;
        % to_plot = RD_map;
        % saving plot data
        % all_to_plot{end+1} = to_plot;
        fprintf('[INFO] processing frame no: %d\n', frameCounter);

        if ~isfolder(frame_folder)
            mkdir(frame_folder);
        end
        save(fullfile(frame_folder, sprintf('frame_%05d.mat', frameCounter)), 'RD_map', '-v7.3');
        frameCounter = frameCounter + 1;

        % --------------- DISPLAY ---------------------------------

        % display_graph(params, to_plot, range_axis, doppler_axis, range_angle_stich, 10);
    end
end 

params.total_frames = frameCounter-1;


save(fullfile(output_folder, "config.mat"), "all_range_axis", "all_doppler_axis", '-v7.3');

save(fullfile(output_folder, [testRootFolder '_params.mat']), 'params', '-v7.3');
