% PMMmap Generator for TX Beamforming project (from output/ saved frames)
clearvars;

data_folder = './output/txbf_13_7_25/';
frame_folder = [data_folder 'rangeDopplerFFTmap_11/'];
config_folder = data_folder;

frame_files = dir(fullfile(frame_folder, 'frame_*.mat'));
numFrames = numel(frame_files);

% Load config for dimension info
params_file = dir(fullfile(config_folder, '*_params.mat'));
assert(~isempty(params_file), 'Cannot find *_params.mat in the given folder');
params = load(fullfile(config_folder, params_file(1).name), 'params');
params = params.params;
anglesToSteer = params.anglesToSteer;
numAngles = params.NumAnglesToSweep;
numRx = params.numRX;

% Load one frame to get size
tmp = load(fullfile(frame_files(1).folder, frame_files(1).name));
RD_map_sample = abs(tmp.RD_map); % [R D Rx Ang]
[numRangeBins, numDopplerBins, ~, ~] = size(RD_map_sample);

% Parameters for spectrum folding
jMin = 2;
jMax = 20;

% Allocate PMMmap: [R Rx Ang F]
PMMmap = zeros(numRangeBins, numRx, numAngles, numFrames);

for f = 1:numFrames
    tmp = load(fullfile(frame_files(f).folder, frame_files(f).name));
    RD_map = abs(tmp.RD_map); % [R D Rx Ang]
    for ang = 1:numAngles
        for rx = 1:numRx
            for r = 1:numRangeBins
                dopplerSpectrum = RD_map(r, :, rx, ang); % 1xD
                bestScore = 0;
                for j = jMin:jMax
                    M = floor(numDopplerBins / j);
                    if M < 1
                        break;
                    end
                    Xcut = dopplerSpectrum(1:(j*M));
                    Xmat = reshape(Xcut, j, M);
                    colAvg = sum(Xmat, 2) ./ M;
                    score_j = max(colAvg);
                    bestScore = max(bestScore, score_j);
                end
                PMMmap(r, rx, ang, f) = bestScore;
            end
        end
    end
    if mod(f,10)==0, fprintf('Processed frame %d/%d\n', f, numFrames); end
end

save(fullfile(data_folder, 'PMMmap.mat'), 'PMMmap', '-v7.3');
disp('Saved PMMmap.mat');
