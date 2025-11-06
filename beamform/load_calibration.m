%-----------------------------------------------------------------------
function calib = load_calibration(phaseShifterCalibPath, mismatchPath, txSel)
% Loads phase‑shifter & mismatch LUTs and trims them to selected TX IDs.

    %% Phase‑shifter (PS) calibration – 63 steps × 12 channels
    S  = load(phaseShifterCalibPath, "Ph");
    PS_actual = squeeze(S.Ph(:,1,:)).';       % → [12 × 63]
    PS_actual(PS_actual>0) = PS_actual(PS_actual>0) - 360;         % map to (‑360,0] range

    calib.psActual = PS_actual(txSel, :);     % keep only used TXs

    %% Per‑channel mismatch LUT (optional)
    if exist(mismatchPath, "file")
        M = load(mismatchPath, "calibResult");
        calib.txMismatch = M.calibResult.TxMismatch;   % [3 × 4]
        calib.rxCal = M.calibResult.RxMismatch;
    else
        calib.txMismatch = zeros(3,4);
        warning("TX mismatch file not found – proceeding with zeros.");
    end
end
