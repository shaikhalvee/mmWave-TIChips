%% tx_beamforming_demo.m
% Full example to recreate TX beamforming beams for:
% - 12-element linear TX row (0.5 lambda spacing)
% - Optional additional subarrays (example positions taken from diagram labels)
% Uses units of lambda in the diagram; converts to meters for phased toolbox.

clear; close all; clc;

%% --- User / radar params ---
fc    = 77e9;                      % carrier frequency (Hz) - set to your radar carrier
c     = physconst('LightSpeed');   % speed of light
lambda = c/fc;                     % wavelength (m)

% Choose geometry: 'tx12' (only 12-element row) or 'full' (12-element row + example subarrays)
geometryType = 'full';

% Steering angles to visualize (azimuth sweep, elevation fixed at 0)
steerAnglesDeg = -60:5:60;    % degrees
 
% Plotting resolution
azGrid = -180:1:180;
elGrid = -90:1:90;

%% --- Build element positions (in units of lambda) ---
% 1) 12-element linear TX row (0.5 lambda spacing along x)
Ntx = 12;
tx_x_wl = (0 : 0.5 : 0.5*(Ntx-1));   % x positions in lambda units (0, 0.5, ..., 5.5)
tx_y_wl = zeros(size(tx_x_wl));
tx_z_wl = zeros(size(tx_x_wl));

% center the row about origin (optional — keeps symmetric plots)
tx_x_wl = tx_x_wl - mean(tx_x_wl);

% 2) Example cascaded/clustered subarrays (based on diagram dotted box)
% - Two 4-element linear subarrays (call them subA and subB) spaced 4 lambda apart
% - Each subarray has 4 elements with 0.5 lambda spacing
sub4_spacing = 0.5;
sub4_elems = 4;
sub4_x_local = (0 : sub4_spacing : sub4_spacing*(sub4_elems-1));
sub4_x_local = sub4_x_local - mean(sub4_x_local); % center each subarray locally

% Place subarrays at some offset to the left of the big TX row (for example -16 lambda)
sep_between_main_and_sub = -16;   % in lambda units (diagram shows ~16)
sub_offset_x = sep_between_main_and_sub;

% place two subarrays separated by 4 lambda (diagram label '4')
sub_sep = 4; % lambda units
subA_x_center = sub_offset_x;
subB_x_center = sub_offset_x + sub_sep;

% Subarray element coordinates (x,y,z in lambda)
subA_x = subA_x_center + sub4_x_local;
subB_x = subB_x_center + sub4_x_local;

% put these subarrays a little above/below in y (to show vertical offset)
subA_y = 0.5 * ones(size(subA_x));   % 0.5 lambda offset in y
subB_y = -0.5 * ones(size(subB_x));

% small stacked cluster (vertical stack) shown in dotted box (example)
% coordinates taken from drawing: vertical spacing [1.5, 0.5, 0.5] etc -> approximate
cluster_x = sub_offset_x + 2;   % place near subarrays
cluster_y = 0; 
cluster_z = [0, 0.5, 1.5, 0.5]; % vertical stack in lambda units (example)
cluster_x = cluster_x + [0, 0, 0, 0]; % same x for stacked elements
cluster_y = cluster_y + [ -1, -1, -1.5, -0.5 ]; % small y offsets example

% Compose final position lists in wavelength units
if strcmpi(geometryType,'tx12')
    pos_wl = [tx_x_wl; tx_y_wl; tx_z_wl];
else
    % combine: main tx row + subarrays + cluster
    pos_wl = [ tx_x_wl, ...
               subA_x, subB_x, ...
               cluster_x ];
    pos_wl_y = [ tx_y_wl, ...
                 subA_y, subB_y, ...
                 cluster_y ];
    pos_wl_z = [ tx_z_wl, ...
                 zeros(1,numel(subA_x)), zeros(1,numel(subB_x)), ...
                 cluster_z ];
    pos_wl = [pos_wl; pos_wl_y; pos_wl_z];
end

% For tx12 only case pos_wl may already be 3xN; ensure shape:
if size(pos_wl,1)~=3
    error('pos_wl should be 3xN (x;y;z) in lambda units.');
end

%% --- Convert to meters (phased toolbox expects meters) ---
pos_m = pos_wl * lambda;  % 3 x N matrix in meters

%% --- Create element and array objects ---
% Use isotropic elements (simple). If you have a real element pattern, replace this.
elem = phased.IsotropicAntennaElement('FrequencyRange',[fc-1e9 fc+1e9]); 

% Use ConformalArray so arbitrary positions are supported
txArray = phased.ConformalArray('Element', elem, 'ElementPosition', pos_m);

%% --- Show array geometry ---
figure('Name','TX Array Geometry');
viewArray(txArray, 'ShowIndex', 'All');
title('TX array geometry (meters)');

%% --- Beamforming weights (simple phase-steering / uniform amplitude) ---
% Compute steering vectors for all desired steering angles (azimuth, elevation)
% We'll compute steering vectors and normalize them (conventional phased array steering).
SV = phased.SteeringVector('SensorArray', txArray, 'PropagationSpeed', c);

% Preallocate
numAngles = numel(steerAnglesDeg);
weights = zeros(size(pos_m,2), numAngles);

for idx = 1:numAngles
    az = steerAnglesDeg(idx);
    el = 0; % elevation = 0 deg (modify if you want elevation steering)
    sv = SV(fc, [az; el]);    % returns column vector length N
    % conventional phase-steering weights (unit amplitude, conjugate of steering vector)
    w = sv ./ norm(sv);      % unit-norm
    weights(:,idx) = w;
end

%% --- Plot azimuth patterns for selected steering angles ---
figure('Name','Azimuth Patterns (elevation=0)');
% Create tiled layout
ncols = 3; nrows = ceil(numAngles/ncols);
tiledlayout(nrows,ncols,'TileSpacing','compact','Padding','compact');

for idx = 1:numAngles
    nexttile;
    % pattern call plots in dB; choose -180:180 az and el=0
    pattern(txArray, fc, azGrid, 0, 'Type','powerdb', 'Weights', weights(:,idx));
    title([num2str(steerAnglesDeg(idx)) char(186)]);
    xlabel('Az (deg)'); ylabel('Gain (dB)');
    axis([-180 180 -60 10]);
end

sgtitle('TX Beampatterns steered to various azimuth angles (elevation = 0°)');

%% --- Plot elevation cut at a selected steering azimuth (e.g., 30 deg) ---
selAz = 30; % chosen steering azimuth
% find index for 30 deg if present
[~, selIdx] = min(abs(steerAnglesDeg - selAz));
if isempty(selIdx), selIdx = 1; end

figure('Name','Elevation Pattern at selected AZ');
pattern(txArray, fc, selAz, elGrid, 'Type','powerdb', 'Weights', weights(:,selIdx));
title(['Elevation pattern at azimuth = ' num2str(selAz) '°']);
xlabel('Elevation (deg)'); ylabel('Gain (dB)');

%% --- Example: Plot 2D polar cut of one beam (cartesian) ---
figure('Name','2D Cartesian Azimuth Cut (one beam)');
pattern(txArray, fc, azGrid, 0, 'Type','powerdb', 'Weights', weights(:,selIdx));
title(['Azimuth cut (elevation=0°) steered to ' num2str(steerAnglesDeg(selIdx)) '°']);

%% --- Save weights / positions for later use ---
save('tx_array_beam_data.mat', 'pos_wl', 'pos_m', 'weights', 'steerAnglesDeg', 'fc', 'lambda');

fprintf('Done. GeometryType="%s". Positions: %d elements. Data saved to tx_array_beam_data.mat\n', geometryType, size(pos_m,2));
