function stats = analyze_corr_concentration_accel( ...
        frame_files, all_range_axis, all_doppler_axis, ...
        jmin, jmax, maxRange, droneOpts, angleIdx)
% ANALYZE_CORR_CONCENTRATION_ACCEL
%
% For each frame:
%   1) Builds RD_slice [R x D] (power) for the chosen angle.
%   2) Applies the same gating logic (compute_gate_for_drone).
%   3) Runs calc_folding_concentration -> P_i, j_best, C_i (=cnctr_ratio).
%   4) Inside the gate, selects the UAV bin = argmax P_i (folding) and
%      records:
%         C_star(f) = concentration ratio at that bin
%         v_star(f) = Doppler velocity (peak of gated spectrum)
%         j_star(f) = j* at that bin
%         r_star(f) = range at that bin
%
% Then builds:
%   dv(f)   = v_star(f) - v_star(f-1)
%   aMag(f) = |dv(f)|
%   dC(f)   = C_star(f) - C_star(f-1)
%
% And computes correlations:
%   - Pearson & Spearman for C* vs v
%   - Pearson & Spearman for C*(2:end) vs dv
%   - Spearman for C*(2:end) vs |dv|
%   - Spearman for dC vs |dv|
%
% Inputs:
%   frame_files      : struct from dir('frame_*.mat')
%   all_range_axis   : cell array {F x 1}, each is range_axis for frame f
%   all_doppler_axis : cell array {F x 1}, each is doppler_axis for frame f
%   jmin, jmax       : folding search range for calc_folding_concentration
%   maxRange         : max range (m) to keep (same as viewer)
%   droneOpts        : struct with fields:
%                        .auto_gate    (logical)
%                        .gate_center_m
%                        .gate_width_m
%                        .v_exclude
%   angleIdx         : which TX beam index to analyze (1..NumAngles)
%
% Output:
%   stats struct with fields:
%       C_star    [F x 1]  concentration at UAV bin per frame
%       v_star    [F x 1]  velocity at UAV bin per frame
%       dv        [F-1 x 1] velocity difference
%       aMag      [F-1 x 1] |dv|
%       dC        [F-1 x 1] difference in C*
%       j_star    [F x 1]  j* at UAV bin
%       r_star    [F x 1]  range at UAV bin
%
%       Rlin_Cv       scalar Pearson C* vs v
%       rhoS_Cv       scalar Spearman C* vs v
%       Rlin_Cdv      scalar Pearson C*(2:end) vs dv
%       rhoS_Cdv      scalar Spearman C*(2:end) vs dv
%       rhoS_C_aMag   scalar Spearman C*(2:end) vs |dv|
%       rhoS_dC_aMag  scalar Spearman dC vs |dv|
%
% NOTE: requires calc_folding_concentration and compute_gate_for_drone
%       to be on the MATLAB path.

    if nargin < 8 || isempty(angleIdx)
        angleIdx = 1;
    end

    F = numel(frame_files);

    C_star = nan(F,1);   % concentration ratio at UAV bin
    v_star = nan(F,1);   % velocity at UAV bin
    j_star = nan(F,1);   % j* at UAV bin
    r_star = nan(F,1);   % range at UAV bin

    % v_res = droneOpts.v_res;
    v_max = droneOpts.v_max;

    for f = 1:F
        tmp = load(fullfile(frame_files(f).folder, frame_files(f).name));

        % --- Get RD map in [R x D x Ang] form ---
        if isfield(tmp,'RD_map')
            if ~isstruct(tmp.RD_map)
                DF = tmp.RD_map;
                if ndims(DF) == 2
                    DF = reshape(DF, size(DF,1), size(DF,2), 1);
                end
                RD = DF;
            else
                if isfield(tmp.RD_map,'dopplerFFT')
                    DF = tmp.RD_map.dopplerFFT;
                elseif isfield(tmp.RD_map,'rangeDopplerMap')
                    DF = tmp.RD_map.rangeDopplerMap;
                else
                    error('RD_map struct missing dopplerFFT/rangeDopplerMap');
                end
                if ndims(DF) == 2
                    DF = reshape(DF, size(DF,1), size(DF,2), 1);
                end
                RD = DF;
            end
        else
            error('Frame file missing RD_map: %s', frame_files(f).name);
        end

        RD_abs = abs(RD).^2;               % [R x D x Ang]

        % Angle slice [R x D]
        angleIdx_use = min(max(1, angleIdx), size(RD_abs,3));
        RD_slice = RD_abs(:,:,angleIdx_use);  % [R x D]

        % Axes
        range_axis   = all_range_axis{f};
        doppler_axis = all_doppler_axis{f};

        % Trim to maxRange (same as viewer)
        idx_range = find(range_axis <= maxRange);
        if isempty(idx_range)
            continue;   % keep NaNs
        end
        RD_slice = RD_slice(idx_range,:);
        range_axis = range_axis(idx_range);

        % Gate for this frame (same logic as viewer)
        [gate_idx, gate_center_m, gate_width_m] = compute_gate_for_drone( ...
            RD_slice, range_axis, doppler_axis, ...
            droneOpts.auto_gate, ...
            droneOpts.gate_center_m, ...
            droneOpts.gate_width_m, ...
            droneOpts.v_exclude);

        if isempty(gate_idx)
            % no valid gate: leave NaNs
            continue;
        end

        % --- Folding + concentration for this frame ---
        [P_fold, j_best, ~, cnctr_ratio, ~, ~] = ...
            calc_folding_concentration(RD_slice, jmin, jmax);

        % Restrict to gate
        P_gate = P_fold(gate_idx);
        C_gate = cnctr_ratio(gate_idx);
        j_gate = j_best(gate_idx);
        r_gate = range_axis(gate_idx);

        % UAV bin = bin with maximum folding P_i inside gate
        [~, idx_maxP] = max(P_gate);
        rbin = gate_idx(idx_maxP);    % index in trimmed axis | UAV range bin

        % Record per-frame quantities at UAV bin
        C_star(f) = cnctr_ratio(rbin);  % concentration ratio at UAV bin
        j_star(f) = j_best(rbin);       % j* at UAV bin
        r_star(f) = range_axis(rbin);   % physical range

        % Simple velocity estimate: peak of mean-gated Doppler
        % mean power over gate, then pick peak Doppler
        doppler_gated = mean(RD_slice(gate_idx, :), 1);   % [1 x D]
        [~, kmax] = max(doppler_gated);
        v_star(f) = doppler_axis(kmax);
    end

    % --------- Build differences and acceleration proxy ----------
    v_range = 2 * v_max;        % taking 2*v_max to handle aliasing
    dv_raw = diff(v_star);      % signed velocity change
    dv = mod(dv_raw + v_range/2, v_range) - v_range/2;        % true velocity change
    aMag = abs(dv);             % magnitude of change (acceleration-like)
    dC = diff(C_star);        % change in concentration

    % Align C* with dv (dv(f) = v(f+1)-v(f) => use C*(2:end))
    C2 = C_star(2:end);

    % --------- Correlation: C* vs v ---------
    validCv = ~isnan(C_star) & ~isnan(v_star);
    if nnz(validCv) > 2
        Rlin = corrcoef(C_star(validCv), v_star(validCv)); % R_C_v 
        Rlin_Cv = Rlin(1,2);
        rhoS_Cv = corr(C_star(validCv), v_star(validCv), 'Type','Spearman');
    else
        Rlin_Cv = NaN;
        rhoS_Cv = NaN;
    end

    % --------- Correlation: C*(2:end) vs dv ---------
    validCdv = ~isnan(C2) & ~isnan(dv);
    if nnz(validCdv) > 2
        Rlin2 = corrcoef(C2(validCdv), dv(validCdv)); % R_C_dv 
        Rlin_Cdv = Rlin2(1,2);
        rhoS_Cdv = corr(C2(validCdv), dv(validCdv), 'Type','Spearman');
    else
        Rlin_Cdv = NaN;
        rhoS_Cdv = NaN;
    end

    % --------- Correlation: C*(2:end) vs |dv| (accel magnitude) ---------
    validCa = ~isnan(C2) & ~isnan(aMag);
    if nnz(validCa) > 2
        rhoS_C_aMag = corr(C2(validCa), aMag(validCa), 'Type','Spearman');
    else
        rhoS_C_aMag = NaN;
    end

    % --------- Correlation: dC vs |dv| ---------
    validdCa = ~isnan(dC) & ~isnan(aMag);
    if nnz(validdCa) > 2
        rhoS_dC_aMag = corr(dC(validdCa), aMag(validdCa), 'Type','Spearman');
    else
        rhoS_dC_aMag = NaN;
    end

    % --------- Optional: quick scatter plots (comment out if not needed) ---------
    % if nnz(validCv) > 2
    %     figure; scatter(v_star(validCv), C_star(validCv), 'filled');
    %     xlabel('Velocity v (m/s)');
    %     ylabel('Concentration C^*');
    %     title(sprintf('C^* vs v  (Spearman \\rho = %.3f)', rhoS_Cv));
    %     grid on;
    % end
    % 
    % if nnz(validCa) > 2
    %     figure; scatter(aMag(validCa), C2(validCa), 'filled');
    %     xlabel('|\\Delta v| (m/s)');
    %     ylabel('Concentration C^*(f)');
    %     title(sprintf('C^* vs |\\Delta v|  (Spearman \\rho = %.3f)', rhoS_C_aMag));
    %     grid on;
    % end

    % --------- Pack outputs ---------
    stats.C_star      = C_star;
    stats.v_star      = v_star;
    stats.dv          = dv;
    stats.aMag        = aMag;
    stats.dC          = dC;
    stats.j_star      = j_star;
    stats.r_star      = r_star;

    stats.Rlin_Cv     = Rlin_Cv;
    stats.rhoS_Cv     = rhoS_Cv;
    stats.Rlin_Cdv    = Rlin_Cdv;
    stats.rhoS_Cdv    = rhoS_Cdv;
    stats.rhoS_C_aMag = rhoS_C_aMag;
    stats.rhoS_dC_aMag= rhoS_dC_aMag;
end
