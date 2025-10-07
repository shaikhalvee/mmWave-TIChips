function [RD_sst, doppler_axis] = reconstruct_RD_from_sstfrft(A_cell, meta, params)
% Rebuild slow-time, then Doppler FFT — per range bin
R   = numel(A_cell);
K   = meta.Ktot;       L = meta.L;     H = meta.H;
Nw  = meta.Nw;         D = meta.D;     w = meta.win;

RD_N = params.dopplerFFTSize;       % same size you use normally
PRT  = (params.Idle_Time_us + params.Ramp_End_Time_us)*1e-6;
lambda = params.lambda;  

RD_sst = zeros(R, RD_N);
dop_win = hann_local(K);

for r = 1:R
    y_hat  = complex(zeros(K,1));
    wsum   = zeros(K,1);
    A = A_cell{r};                    % [M x Nw] (sparse)

    for wi = 1:Nw
        idx = (wi-1)*H + (1:L);
        if idx(end) > K, idx = K-L+1:K; end
        xw_hat = D * full(A(:,wi));   % reconstruct window (already windowed)
        y_hat(idx) = y_hat(idx) + xw_hat;
        wsum(idx)  = wsum(idx) + abs(w).^2;
    end
    y_hat = y_hat ./ max(wsum, eps);  % overlap–add normalization

    % Doppler FFT exactly like your pipeline
    Yd = fftshift(fft(y_hat .* dop_win, RD_N, 1), 1);
    RD_sst(r, :) = abs(Yd).';         % magnitude for heatmap
end

% Doppler/velocity axis to match your code
v_max = lambda / (4 * PRT);
doppler_axis = linspace(-v_max, v_max, RD_N);
end
