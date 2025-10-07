%% Extract TX coordinates from the TI profile and plot 3D beam
clear; clc;

% ---- Paste your function into the path and call it (optionally pass angles) ----
params = chirpProfile_TxBF_LRR(0);  % or chirpProfile_TxBF_LRR(-30:2:30)

% Design and center frequencies
f_design = params.TI_Cascade_Antenna_DesignFreq * 1e9;   % 76.8 GHz in Hz
fc       = (params.Start_Freq_GHz) * 1e9;                % ~77 GHz in Hz
c        = physconst('LightSpeed');
lambda_c = c/fc;

% TI nominal TX positions in "half-wavelength @ f_design" grid units
azi_idx = params.TI_Cascade_TX_position_azi(:);   % length 12
ele_idx = params.TI_Cascade_TX_position_ele(:);   % length 12

% Which TXs are enabled for BF
sel = params.Tx_Ant_Arr_BF(:);                    % indices into the 12 TXs

% Scale factor: converts indices to wavelengths at fc
d = 0.5 * (fc / f_design);

% --- Physical coordinates (meters) for the selected TXs ---
x = (azi_idx(sel) * d) * lambda_c;    % along azimuth axis
y = (ele_idx(sel) * d) * lambda_c;    % elevation offsets
z = zeros(size(x));                   % all on the same PCB plane

% Visualize element layout
figure; plot(x/lambda_c, y/lambda_c, 's', 'MarkerFaceColor',[0.85 0 0], 'MarkerEdgeColor','k');
grid on; axis equal;
xlabel('x / \lambda_c'); ylabel('y / \lambda_c');
title('MMWCAS-RF-EVM: TX element coordinates from TI config');

% Build array and plot 3D pattern at broadside
elem = phased.CosineAntennaElement( ...
    'FrequencyRange', [fc-5e9 fc+5e9], ...
    'CosinePower', [6 12]);  % [azExp, elExp] – tune to adjust beamwidth/front-to-back
txArray = phased.ConformalArray('Element', elem, 'ElementPosition', [x.'; y.'; z.']);

% Broadside steering weights
sv = phased.SteeringVector('SensorArray', txArray, 'IncludeElementResponse', true);
w  = sv(fc, [0; 0]);  % [az;el] degrees

% 3D pattern
az = -90:0.5:90; el = -90:0.5:90;
figure;
pattern(txArray, fc, az, el, 'Type','directivity', ...
        'CoordinateSystem','polar', 'Weights', w(:));
title('TX pattern with directional (front) element');


% figure;
% pattern(txArray, fc, 'Type','directivity', 'CoordinateSystem','polar', 'Weights', w);
% title('TX beamforming pattern @ broadside');

% Useful 2D cuts
% --- 2D AZIMUTH CUT (el = 0°) ---
figure;
pattern(txArray, fc, -90:0.25:90, 0, ...           % az sweep, el=0
        'Type','directivity', ...
        'CoordinateSystem','rectangular', ...
        'PropagationSpeed', physconst('LightSpeed'), ...
        'Weights', w(:));
title('Azimuth cut @ el = 0°');

% --- 2D ELEVATION CUT (az = 0°) ---
figure;
pattern(txArray, fc, 0, -90:0.25:90, ...           % az=0, el sweep
        'Type','directivity', ...
        'CoordinateSystem','rectangular', ...
        'PropagationSpeed', physconst('LightSpeed'), ...
        'Weights', w(:));
title('Elevation cut @ az = 0°');
