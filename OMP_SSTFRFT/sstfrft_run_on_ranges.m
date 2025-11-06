function [A_cell, C_cell, tf_cell, meta] = sstfrft_run_on_ranges(x_rk, PRF, cfg)
% x_rk: [R, K] complex slow-time per range bin
% Returns:
%   C_cell{r}: |coeffs| for window centers x (f0, mu)  => [Nf, Nmu, Nwin]
%   tf_cell{r}: simple 2D time–freq map (max over mu)  => [Nf, Nwin]
%   meta.win_centers_idx, meta.t_axis_s, meta.f0_hz, meta.mu_hz_s

[R, K] = size(x_rk);

L   = cfg.win_len_chirps;                  % window length (samples)
H   = cfg.hop_chirps;                      % hop
Nw  = 1 + floor((K - L)/H);                % number of windows
w   = hann_local(L);                       % ST window

% time centers (in chirp index & seconds)
centers = (0:Nw-1)*H + floor(L/2);
t_axis_s = (centers - 1)/PRF;

% Prebuild dictionary for given grids (shared across ranges & windows)
D = make_chirp_dictionary(L, 1/PRF, cfg.f0_hz, cfg.mu_hz_s, w);  % [L, M]
Dt_norms = sqrt(sum(abs(D).^2,1)); 
D = D./Dt_norms;              % normalize columns
M = size(D,2); % M = Nf * Nmu

A_cell = cell(R,1);   % <— NEW: sparse coeff matrices [M x Nw] per range
C_cell = cell(R,1); % full time–(f₀, μ) coefficient cube for range bin r
tf_cell = cell(R,1); % time–frequency map (max over μ)

for r = 1:R
    xr = x_rk(r, :).';                 % [K x 1]
    if cfg.use_dc_hp
        xr = xr - mean(xr);
    end

    C_all = zeros(numel(cfg.f0_hz), numel(cfg.mu_hz_s), Nw);  % magnitude coeffs
    TFmap = zeros(numel(cfg.f0_hz), Nw);                      % mu-max-projection

    for widx = 1:Nw
        sIdx = (widx-1)*H + (1:L);     % window slice
        if sIdx(end) > K, sIdx = K-L+1:K; end
        xw   = xr(sIdx) .* w;          % apply window

        % OMP sparse coding: xw ≈ D * a, with at most K atoms
        a = omp_sparse(D, xw, cfg.K);

        % INSIDE the loop over windows (after "a = omp_sparse(D, xw, cfg.K);"):
        if widx == 1, A = spalloc(M, Nw, cfg.K*Nw); end
        A(:, widx) = sparse(a);   % keep phases

        % reshape coefficients into (f0, mu) grid, store magnitude
        Aabs = abs(reshape(a, [numel(cfg.f0_hz), numel(cfg.mu_hz_s)]));
        C_all(:,:,widx) = Aabs;

        % quick 2D time–freq map: max over mu
        TFmap(:, widx) = max(Aabs, [], 2);
    end

    C_cell{r} = C_all;        % [Nf, Nmu, Nw]
    tf_cell{r} = TFmap;       % [Nf, Nw]
    A_cell{r} = A;  % Store sparse coefficient matrix for range r
end

meta.win_centers_idx = centers;
meta.t_axis_s        = t_axis_s;
meta.f0_hz           = cfg.f0_hz;
meta.mu_hz_s         = cfg.mu_hz_s;
meta.L    = L; 
meta.H    = H; 
meta.Nw   = Nw;
meta.Ktot = K;            % total chirps in the CPI
meta.win  = w;            % slow-time window used
meta.D    = D;            % dictionary used (shared across ranges)
end
