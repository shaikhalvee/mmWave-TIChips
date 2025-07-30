%   COMPUTE_PMM_MAP   Computes the PMM map using spectrum folding.
%
%   PMMmap = compute_PMM_map(fft_complex_radar_cube, jMin, jMax)
%
%   fft_complex_radar_cube : [R x D x Rx x F] (range, doppler, rx, frames)
%   jMin, jMax             : Min/max fold size for spectrum folding
%   PMMmap                 : [R x F x Rx] PMM score map
%
%   Each PMMmap(r, f, rx) contains the best (max) score from spectrum
%   folding the Doppler spectrum at that range bin, frame, and RX.
%

function PMMmap = calc_PMM_beamform(fft_complex_radar_cube, jMin, jMax)
    if nargin < 2, jMin = 2; end
    if nargin < 3, jMax = 20; end

    [numRangeBins, numDopplerBins, numRx, numFrames] = size(fft_complex_radar_cube);

    PMMmap = zeros(numRangeBins, numFrames, numRx);

    for f = 1:numFrames
        for rx = 1:numRx
            for r = 1:numRangeBins
                % Doppler spectrum (magnitude), flip if needed for convention
                dopplerSpectrum = fliplr(abs(fft_complex_radar_cube(r, :, rx, f)));

                bestScore = 0;
                for j = jMin:jMax
                    M = floor(numDopplerBins / j);
                    if M < 1, break; end

                    % Reshape to j x M
                    Xcut = dopplerSpectrum(1:(j*M));
                    Xmat = reshape(Xcut, j, M);

                    % Column-wise average, then take the max as the score
                    colAvg = sum(Xmat, 2) ./ M;
                    score_j = max(colAvg);

                    bestScore = max(bestScore, score_j);
                end
                  PMMmap(r, f, rx) = bestScore;
            end
        end
    end

end