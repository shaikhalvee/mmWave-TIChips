%----------------------------------------------------------------------
% TX Beamforming Range-Doppler Processing (single angle, long-range)
%----------------------------------------------------------------------
close all; clc; clearvars;


% ----------------- USER CONFIG ---------------------------------------
adc_data_folder = 'G:\RADAR_DATA\out_txbf_27_10_sl_8_fr_100_2';
[~, testRootFolder, ~] = fileparts(adc_data_folder);
output_folder =  ['./output/' testRootFolder];
oldParamsFile = [output_folder filesep testRootFolder '_params.mat'];
frame_folder = [output_folder filesep 'rangeDopplerFFTmap_11/'];
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
dcOffsetRemoval = true;
dopplerClutterRemoval = true;

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

frameCounter = 1;

for i_file = 1:numel(fileIdx_unique)
    [fileNameStruct] = getBinFileNames_withIdx(adc_data_folder, fileIdx_unique{i_file});
    [numValidFrames, ~] = getValidNumFrames(fullfile(adc_data_folder, fileNameStruct.masterIdxFile));

    for frameId = 1:numValidFrames
        params.frameId = frameId;

        % ----------------- LOAD RAW ADC DATA (ALL RX, 1 ANGLE) ----------
        radar_data_txbf = fcn_read_AdvFrmConfig_BF_Json(fileNameStruct, params);
        radar_data_txbf = radar_data_txbf(:,:,params.TI_Cascade_RX_ID,:);

        % --------------- RANGE-DOPPLER PROCESSING & PLOT ---------------
        [RD_map, range_axis, doppler_axis, ...
            range_angle_stich, params] = calc_range_doppler_bmfrm( ...
                    radar_data_txbf, params, BF_MIMO_ref, dcOffsetRemoval, dopplerClutterRemoval);

        % Store for saving later
        % all_RD_map{end+1} = RD_map;
        all_range_axis{end+1} = range_axis;
        all_doppler_axis{end+1} = doppler_axis;
        all_range_angle_stich{end+1} = range_angle_stich;
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


save(fullfile(output_folder, "config.mat"), "all_range_axis", "all_doppler_axis", "all_range_angle_stich", '-v7.3');

save(fullfile(output_folder, [testRootFolder '_params.mat']), 'params', '-v7.3');


function display_graph(params, to_plot, range_axis, doppler_axis, range_angle_stich, startRangeIdx)

    % to_plot is with (R,D,Rx,numAng)

    maxRange = 100; %10 meters
    maxRangeIndex = min(ceil(maxRange/params.rangeBinSize), params.rangeFFTSize);

    frameId = params.frameId;
    logged_plot = 20*log10(to_plot);

    figure(1);
    set(1, 'Name', ['Frame ID:' num2str(frameId)]);

    for angleIdx = 1:params.NumAnglesToSweep
        % ---------------- RD SPECTRUM PER ANGLE -----------------
        rd_map_per_angle = to_plot(:,:,:,angleIdx); % picking the angle dimension
        rd_map_per_angle = mean(rd_map_per_angle, 3); % taking mean across Rx
        subplot(2,2,1);
        imagesc(doppler_axis, range_axis(startRangeIdx:maxRangeIndex), 20*log10(rd_map_per_angle(startRangeIdx:maxRangeIndex,:)));
        axis xy;
        xlabel('Doppler (m/s)'); ylabel('Range (m)');
        title(sprintf('Range-Doppler Map, Angle = %d° (Frame %d)', ...
            params.anglesToSteer(angleIdx), frameId));
        colorbar;
        drawnow;

        % --------------- RANGE PROFILE (new code) ----------------
        subplot(2,2,2)
        plot(range_axis(startRangeIdx:maxRangeIndex), 20*log10(rd_map_per_angle(startRangeIdx:maxRangeIndex,:)), 'LineWidth', 1.5);
        xlabel('Range (m)');
        ylabel('Power (dB)');
        title(sprintf('Range Profile, Frame %d', frameId));
        grid on;

        % --------------- DOPPLER PROFILE (new code) ----------------
        subplot(2,2,3)
        plot(doppler_axis, 20*log10(rd_map_per_angle(startRangeIdx:maxRangeIndex,:)), 'LineWidth', 1.5);
        xlabel('Velocity (m/s)');
        ylabel('Power (dB)');
        title(sprintf('Doppler Profile, Frame %d', frameId));
        pause(0.1);
    end
    
    
    
    % subplot(2,2,1);
    % imagesc(doppler_axis, range_axis(startRangeIdx:maxRangeIndex), to_plot(startRangeIdx:maxRangeIndex,:));
    % axis xy; xlabel('Doppler (m/s)'); ylabel('Range (m)');
    % title(sprintf('Range-Doppler map, Frame %d', frameId));
    % colorbar; drawnow;



    % --------------- 3D PLOT FOR ANGLE SWEEP ---------------------
    numAngles = params.NumAnglesToSweep;
    anglesToSteer = params.anglesToSteer;
    indices_1D = (startRangeIdx: params.Samples_per_Chirp) ;
    if (numAngles) > 1
        sine_theta = sind(anglesToSteer);
        cos_theta = sqrt(1-sine_theta.^2);
        [R_mat, sine_theta_mat] = meshgrid(indices_1D*params.rangeBinSize, sine_theta);
        [~, cos_theta_mat] = meshgrid(indices_1D,cos_theta);
        x_axis = R_mat.*cos_theta_mat;
        y_axis = R_mat.*sine_theta_mat;

        range_angle_stich = (range_angle_stich(indices_1D,:).');
        % range_angle_stich_flipped = flipud(range_angle_stich(indices_1D,:).');
        % subplot(2,2,3);
        % surf(y_axis, x_axis, abs(range_angle_stich).^0.2,'EdgeColor','none');
        % %xlim([-5 5])
        % %ylim([0 10]);
        % view(2);
        % xlabel('meters')
        % ylabel('meters')
        % title('stich range/azimuth')

        subplot(2,2,4);
        surf(y_axis, x_axis, abs(range_angle_stich).^0.2,'EdgeColor','none');
        %xlim([-5 5])
        %ylim([0 10]);
        view(0, 60);
        xlabel('meters')
        ylabel('meters')
        title('stich range/azimuth')
    end
    pause(0.1);
end
