%  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
%
%
%   Redistribution and use in source and binary forms, with or without
%   modification, are permitted provided that the following conditions
%   are met:
%
%     Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%
%     Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the
%     documentation and/or other materials provided with the
%     distribution.
%
%     Neither the name of Texas Instruments Incorporated nor the names of
%     its contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.
%
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
%   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
%   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
%   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
%   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
%   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
%   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
%


%% Calculates phase shifter values for beam steering (TI semantics)
% BEAMSTEERPHASECALC  Quantizes ideal per-TX phases to 6-bit phase-shifter codes.
%
% Inputs:
%   b_ang     [nAng x 1]  Desired steering angles in DEGREES.
%   PS_actual [nTx  x 63] Measured/actual phase per TX for codes 1..63 (DEGREES)
%                         from your calibration file. Note this is NOT the
%                         programmed phase; it is what the hardware actually
%                         produces for each code.
%   D_TX      [1    x nTx] TX element positions in units of HALF-WAVELENGTH
%                         at the TI cascade design frequency (e.g., [0 4 8]).
%   d         scalar      Electrical-spacing scale factor:
%                         d = 0.5 * (centerFrequency / TI_Cascade_Antenna_DesignFreq).
%                         At the design frequency, d = 0.5.
%
% Outputs:
%   slope     [nAng x nTx] Ideal continuous phase (DEGREES) for each angle/TX
%                          before quantization (wrapped to 0..360).
%   PS_Tx     [nTx  x nAng] PROGRAMMED phase values (DEGREES) picked from the
%                          allowed 6-bit grid (0:5.625:354.375). One value per
%                          TX per steering angle.
%   PS_forAoA [nTx  x nAng] ACTUAL phase values (DEGREES) looked up from the
%                          calibration LUT row corresponding to the chosen code.
%
% Notes on selection rule (matches TI’s original helper):
%   • Odd TX indices (1,3,5,...) choose the first LUT entry whose phase is
%     GREATER THAN OR EQUAL to the ideal phase (i.e., the first ≥ “ceil on that side”).
%   • Even TX indices (2,4,6,...) choose the last LUT entry whose phase is
%     LESS THAN OR EQUAL to the ideal phase (i.e., the last ≤ “floor on that side”).
%   • The LUT is constructed as [0, -PS_actual] to include code 0 and match TI’s
%     sign convention (measured values are stored negated).
%   • If each LUT row (after unwrapping) is monotonic increasing, this logic
%     is equivalent to picking the nearest code on the chosen side. Small
%     non-monotonic dips can make it differ from a strict nearest-on-side rule.
%
% All angles/phases here are in DEGREES.

function [slope,PS_Tx,PS_forAoA] = beamSteerPhaseCalc(b_ang, PS_actual, D_TX, d)

    % Number of TX elements (one LUT row per TX)
    numTX = size(PS_actual, 1);

    % Allowed programmed phase grid for the 6-bit shifter (64 codes, LSB = 5.625°)
    PS_allowed = 0 : 5.625 : (360 - 5.625);  % [1 x 64], degrees

    % Build per-TX LUT rows in TI’s convention:
    %   measured phases are negated, and code 0 has phase 0° at the front.
    actualPS10 = -PS_actual;                        % negate measured values
    actualPS10 = [zeros(numTX,1) actualPS10];       % prepend code 0 → [nTx x 64]

    % Pre-allocate outputs:
    PS_Tx     = zeros(numTX, length(b_ang));        % programmed phases (by TX, angle)
    PS_forAoA = zeros(numTX, length(b_ang));        % achieved phases from LUT
    slope     = zeros(length(b_ang), numTX);        % ideal continuous phases

    % Loop over desired steering angles
    for t = 1:length(b_ang)

        % Ideal continuous phase per TX (DEGREES), wrapped to [0,360):
        %   phi_ideal(θ, k) = wrapTo360( 180 * D_TX(k) * d * sin(θ) )
        % Explanation:
        %   • 2π (rad) ↔ 360°; D_TX is in half-λ, so 2π * (D_TX/2) = π*D_TX ⇒ 180*D_TX degrees.
        %   • ‘d’ scales electrical spacing when centerFrequency ≠ design frequency.
        slope(t,:) = wrapTo360( sin(b_ang(t) * 2*pi * d / 180) * D_TX * 180 );

        % --- ODD TX indices (1,3,5,...) → choose the FIRST LUT value ≥ ideal
        for u = 1:2:numTX
            temp = actualPS10(u,:);                              % [1 x 64], degrees
            idx  = find( (temp - slope(t,u)) >= 0, 1, 'first' ); % first ≥ ideal
            if isempty(idx)
                % Fallback (rare): if all LUT entries < ideal, use the first entry (code 0).
                % With 0° present at index 1 and slope∈[0,360), an empty set is unlikely.
                idx = 1;
            end
            PS_forAoA(u,t) = temp(idx);          % actual phase from LUT
            PS_Tx(u,t)     = PS_allowed(idx);    % programmed phase (grid)
        end

        % --- EVEN TX indices (2,4,6,...) → choose the LAST LUT value ≤ ideal
        for u = 2:2:numTX
            temp = actualPS10(u,:);                              % [1 x 64], degrees
            idx  = find( (temp - slope(t,u)) <= 0, 1, 'last' );  % last ≤ ideal
            % ‘idx’ should always exist because code 0 (=0°) ≤ any slope in [0,360).
            PS_forAoA(u,t) = temp(idx);          % actual phase from LUT
            PS_Tx(u,t)     = PS_allowed(idx);    % programmed phase (grid)
        end
    end
end
