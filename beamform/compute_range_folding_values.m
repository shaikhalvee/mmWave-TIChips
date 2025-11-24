function [P, j_best, F_by_j] = compute_range_folding_values(RD_slice, jmin, jmax)
% COMPUTE_RANGE_FOLDING_VALUES
% RD_slice: [R x D] power (or magnitude^2) for ONE angle (your to_plot)
% Returns:
%   P      : [R x 1] folding value per range bin (max over j)
%   j_best : [R x 1] argmax folding size per bin
%   F_by_j : [R x (jmax-jmin+1)] optional: Fi(j) across j for analysis
%
% Implements Eq. (10)-(11): Fi(j) = max_k ( (1/M) sum_{m=1..M} Di[k+(m-1)j] ),
% with M = floor(L/j); Pi = max_{jmin..jmax} Fi(j).
    [R, L] = size(RD_slice);
    Jvals  = jmin:jmax;
    nJ     = numel(Jvals);
    P      = zeros(R,1);
    j_best = zeros(R,1);
    if nargout >= 3, F_by_j = zeros(R, nJ); else, F_by_j = []; end

    for r = 1:R
        Di = RD_slice(r, :);  % 1 x L  Doppler spectrum (power) for range bin r
        best_val = -Inf; best_j = Jvals(1);
        for jj = 1:nJ
            j = Jvals(jj);
            M = floor(L/j);
            if M < 1, val = -Inf; else
                % Build j columns by averaging every j-th element over M rows
                % Column k takes indices k, k+j, k+2j, ... k+(M-1)j
                col_means = zeros(1, j);
                for k = 1:j
                    idx = k + (0:(M-1))*j;
                    col_means(k) = mean(Di(idx));
                end
                val = max(col_means);  % Fi(j)
            end
            if nargout >= 3, F_by_j(r, jj) = val; end % F_i(j) for bin r, fold j
            if val > best_val
                best_val = val; best_j = j;
            end
        end
        P(r) = best_val;   % Pi
        j_best(r) = best_j;     % argmax j
    end
end

