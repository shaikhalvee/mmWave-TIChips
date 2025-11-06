% -----------
% Fills in defaults for the 
% slow-time frequency grid f_0 and chirp-rate grid μ (frequency slope)
% -----------

function cfg = sstfrft_autogrids(cfg, PRF, Nchirps)
% Fill in grids if user left them empty.
% f0_hz in [-PRF/2, +PRF/2]; chirp-rate grid scaled to % of span.

if isempty(cfg.f0_hz) % f0_hz in [-PRF/2, +PRF/2]
    fmax = 0.5*PRF;
    cfg.f0_hz = linspace(-fmax, +fmax, cfg.Nf);
end

if isempty(cfg.mu_hz_s) % μ in Hz/s of slow time (Doppler). Sized to expected RPM drift (narrow if RPM steady)
    fspan = max(cfg.f0_hz) - min(cfg.f0_hz); % basically fspan = PRF
    muMax = cfg.mu_frac_of_f0span * (fspan) / ( (cfg.win_len_chirps-1) / PRF );
    cfg.mu_hz_s = linspace(-muMax, +muMax, cfg.Nmu); % Nmu = 25, a middle number or a balance point
    % Lets the model track frequency drift within the short window 
    % (RPM ramps, aspect change). If RPM is steady, this can be narrow 
    % around zero; if RPM changes, widen it.
end

cfg.Nchirps = Nchirps;
end

% This should ensure the window length fits within the per angle CPI or
% (nchirp_loops)
