function [P, j_best, F_by_j, S, S_norm, mu_best_all] = calc_smearing_degree(RD_slice, jmin, jmax)
% RD_slice: [R x D] power (or |FFT|^2) for ONE angle
%
% Outputs:
%   P           : [R x 1] folding value per range bin (max over j)
%   j_best      : [R x 1] argmax folding size per range bin
%   F_by_j      : [R x (jmax-jmin+1)] Fi(j) for all j (optional table)
%   S           : [R x 1] smearing degree S_i
%   S_norm      : [R x 1] normalized smearing S_i / j_i^*
%   mu_best_all : {R x 1} cell array, each cell is μ_D(j_i^*) (1 x j_i^*)

    [R, L] = size(RD_slice);
    Jvals  = jmin:jmax;
    nJ     = numel(Jvals);

    P           = zeros(R,1);
    j_best      = zeros(R,1);
    S           = zeros(R,1);
    S_norm      = zeros(R,1);
    F_by_j      = -inf(R, nJ);
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
                val       = -Inf;
                col_means = [];
            else
                % μ_k(j): columnwise averages (this is your μ_D(j))
                col_means = zeros(1, j);
                for k = 1:j
                    idx = k + (0:(M-1))*j;
                    col_means(k) = mean(Di(idx));
                end
                val = max(col_means);      % F_i(j)
            end

            F_by_j(r, jj) = val;

            if val > best_val
                best_val = val;
                best_j   = j;
                mu_best  = col_means;      % store μ_D(j_best) for this range
            end
        end

        % Folding results
        P(r)           = best_val;
        j_best(r)      = best_j;
        mu_best_all{r} = mu_best;          % may be [] if nothing valid

        % ---- smearing from μ_D(j_best) ----
        if isempty(mu_best)
            S(r)      = 0;
            S_norm(r) = 0;
        else
            mu     = mu_best;
            j      = best_j;
            mu_bar = mean(mu);
            [~, p] = max(mu);              % PMM column index

            % left
            N_left = 0; kk = p - 1;
            while kk >= 1 && mu(kk) >= mu_bar
                N_left = N_left + 1;
                kk = kk - 1;
            end
            % right
            N_right = 0; kk = p + 1;
            while kk <= j && mu(kk) >= mu_bar
                N_right = N_right + 1;
                kk = kk + 1;
            end

            S(r)      = N_left + N_right;
            S_norm(r) = S(r) / j;
        end
    end
end
