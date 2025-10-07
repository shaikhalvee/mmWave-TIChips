%-----------------------------------------------------------------------
function [idealDeg, psTx, psLut] = beamSteerPhaseCalcOpt(bAng, psActual, dTx, d)
% Vectorised rewrite of TI's helper (no odd/even loops, no "find").
% Returns:
%   idealDeg – ideal continuous phase for each TX (nAng × nTx)
%   psTx     – programmed phase (deg) after quantisation (nTx × nAng)
%   psLut    – actual achieved phase from LUT (nTx × nAng)

    nTx   = size(psActual,1);
    nAng  = numel(bAng);
    LSB   = 5.625;                              % deg
    psAllowed = 0:LSB:360-LSB;                  % 64 values

    % Ideal continuous phase (deg) ------------------------------
    % Compute ideal phase in DEGREES for each (angle, tx)
    % bAng: column vector [nAng x 1], dTx: row vector [1 x nTx]
    arg = bAng(:) * 2 * pi * d / 180;  % [nAng x 1] * scalar → [nAng x 1]
    idealDeg = wrapTo360( sin(arg) * dTx * 180 ); % broadcasting → (nAng × nTx)

    % Build LUT look‑up matrix ----------------------------------
    lut = [zeros(nTx,1) -psActual];             % prepend 0, negate sign

    % For each TX, use interp1 to pick nearest LUT value ≥ / ≤ ideal
    psTx  = zeros(nTx, nAng);
    psLut = zeros(nTx, nAng);

    for tx = 1:nTx
        % Vectorised distance between ideal and LUT for this TX
        delta = lut(tx,:) - idealDeg(:,tx);     % (nAng × 64)

        % Odd TX → choose min positive delta (≥0). Even TX → max negative.
        if mod(tx,2) % odd index
            % posIdx = delta >= 0;
            % [~,col] = min(delta .* posIdx + ~posIdx*Inf, [], 2);
            posIdx = delta >= 0;
            deltaMasked = delta;
            deltaMasked(~posIdx) = Inf;
            [~,col] = min(deltaMasked, [], 2);
            % No special handling needed, col=1 is fine if all Inf
            % So, already matches TI logic: use index 1
        else    % even index
            % negIdx = delta <= 0;
            % [~,col] = max(delta .* negIdx - ~negIdx*Inf, [], 2);
            negIdx = delta <= 0;
            deltaMasked = delta;
            deltaMasked(~negIdx) = -Inf;
            [~,col] = max(deltaMasked, [], 2);
            % If all -Inf (i.e., all negIdx false), col==1 (first index)
            % But original code wants 'last' index
            % So correct all-false case:
            for ii = 1:nAng
                if all(~negIdx(ii,:)) % all values -Inf
                    col(ii) = size(lut,2); % last index
                end
            end
        end
        psTx(tx,:)  = psAllowed(col).';
        psLut(tx,:) = lut(tx,col).';
    end
end
