%-----------------------------------------------------------------------
function [Tx1_deg, Tx2_deg, Tx3_deg] = build_phase_luts(params, calib)
% Generates **hardware‑ready** phase tables (°) for each steering angle.

    % Ideal → discrete PS lookup per selected TX
    [~, psTx, ~] = beamSteerPhaseCalc(params.anglesToSteer, ...
        calib.psActual, params.D_TX_BF, params.d_BF);            % → [numTx × nAng]

    % Map back to 12‑slot cascade layout (zeros for unused TXs)
    nAng = numel(params.anglesToSteer);
    ps12 = zeros(nAng, 12);          % [nAng × 12]
    ps12(:, params.Tx_Ant_Arr_BF) = psTx.';                 % transpose!

    % Reshape to device order (3 TX/ASIC × 4 ASICs)
    psDev = reshape(ps12.', 3, 4, nAng);                 % [3 × 4 × nAng]

    % Add per‑device mismatch LUT, wrap to 0–360°
    psDev = wrapTo360(psDev + calib.txMismatch);

    % Export as (nAng × 4) matrices per TX index (angles, devices)
    Tx1_deg = permute(psDev(1, :, :), [3, 2, 1]); % [nAng × 4 × 1] -> [nAng × 4]
    Tx2_deg = permute(psDev(2, :, :), [3, 2, 1]);
    Tx3_deg = permute(psDev(3, :, :), [3, 2, 1]);
end
