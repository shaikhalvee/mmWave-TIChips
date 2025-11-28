function [P, j_best, F_by_j, cnctr_ratio, leak_ratio, mu_best_all] = calc_folding_concentration(RD_slice, jmin, jmax)
% RD_slice: [R x D] power (or |FFT|^2) for ONE angle
%
% Outputs:
%   P           : [R x 1] folding value per range bin (max over j)
%   j_best      : [R x 1] argmax folding size per range bin
%   F_by_j      : [R x (jmax-jmin+1)] F_i(j) for all j (optional table)
%   S           : [R x 1] concentration ratio:
%                   S(i) = μ_p / mean(μ_rest)
%   S_norm      : [R x 1] leakage-style ratio (inverse contrast):
%                   S_norm(i) = mean(μ_rest) / μ_p
%   mu_best_all : {R x 1} cell array, each cell is μ_D(j_i^*) (1 x j_i^*)
%
% where for each range bin i at j = j_i^*:
%   μ = μ_D(j_i^*) is the columnwise average array,
%   p = argmax_k μ_k is the PMM column,
%   μ_p = μ(p),
%   μ_rest = μ(k ≠ p), and mean(μ_rest) is the average of the other columns.

    [R, L] = size(RD_slice);
    Jvals = jmin:jmax;
    nJ = numel(Jvals);

    if jmin <= 1
        error('calc_smearing_degree:InvalidJmin', ...
            'jmin must be >= 2 to compute concentration ratio.');
    end

    P = zeros(R,1);
    j_best = zeros(R,1);
    cnctr_ratio = zeros(R,1);
    leak_ratio = zeros(R,1);
    F_by_j = -inf(R, nJ);
    mu_best_all = cell(R,1);

    for r = 1:R
        Di = RD_slice(r, :);      % 1 x L

        best_val = -Inf;
        best_j   = Jvals(1);
        mu_best  = [];

        % ---- folding scan over j ----
        for jj = 1:nJ
            j = Jvals(jj);
            M = floor(L/j);

            if M < 1
                val = -Inf;
                col_means = [];
            else
                % μ_k(j): columnwise averages (μ_D(j))
                col_means = zeros(1, j);
                for k = 1:j
                    idx = k + (0:(M-1))*j;
                    col_means(k) = mean(Di(idx));
                end
                val = max(col_means);    % F_i(j)
            end

            F_by_j(r, jj) = val;

            if val > best_val
                best_val = val;
                best_j   = j;
                mu_best  = col_means;    % store μ_D(j_best) for this range
            end
        end

        % Folding results
        P(r) = best_val;
        j_best(r)= best_j;
        mu_best_all{r} = mu_best;        % may be [] if nothing valid

        % ---- concentration / leakage ratios from μ_D(j_best) ----
        if isempty(mu_best)
            cnctr_ratio(r) = 0;
            leak_ratio(r) = 0;
        else
            mu = mu_best;
            j = best_j;

            % PMM / main column
            [~, p] = max(mu);
            mu_p = mu(p);

            % Average of all *other* columns
            mu_rest = mu;
            mu_rest(p) = [];
            mu_rest_mean = mean(mu_rest);

            % High S => highly concentrated in main column
            cnctr_ratio(r) = mu_p / mu_rest_mean;

            % High S_norm => more leakage / smearing (inverse contrast)
            leak_ratio(r) = mu_rest_mean / mu_p;
        end
    end
end
