function y = frft_m(x, a)
%FRFT  Fractional Fourier Transform (1-D), robust & simple.
%   y = frft(x, a) computes the fractional Fourier transform of vector x
%   of order a (real). Special cases:
%       a = 0 -> identity
%       a = 1 -> FFT (unitary, centered via fftshift/ifftshift)
%       a = 2 -> time reversal
%       a = 3 -> IFFT (unitary, centered)
%
%   Notes:
%   - Returns a column vector if x is column, a row if x is row.
%   - General case uses a discrete FRFT kernel (direct sum).
%   - Normalization is chosen to be unitary-like; absolute scale may differ
%     from other references, but relative spectra are consistent.

    % ensure vector
    if ~isvector(x), error('frft_m: x must be 1-D.'); end
    col = iscolumn(x);
    x = x(:); N = length(x);
    if N == 0, y = x; if ~col, y=y.'; end; return; end

    % reduce order to [0,4)
    a = mod(real(a), 4);

    % special cases
    if abs(a) < 1e-12   % a=0
        y = x; if ~col, y=y.'; end; return;
    elseif abs(a - 1) < 1e-12   % a=1
        y = fftshift(fft(ifftshift(x))) / sqrt(N);
        if ~col, y=y.'; end; return;
    elseif abs(a - 2) < 1e-12   % a=2
        y = flipud(x);
        if ~col, y=y.'; end; return;
    elseif abs(a - 3) < 1e-12   % a=3
        y = fftshift(ifft(ifftshift(x))) * sqrt(N);
        if ~col, y=y.'; end; return;
    end

    % map remaining cases to (0,2)
    if a > 2
        % F^{a} = F^{a-2} o F^{2}; F^{2} is time reversal
        y = frft_m(flipud(x), a - 2);        % FIXED
        if ~col, y = y.'; end; return;
    elseif a < 0    % unreachable
        % F^{-a} = (F^{a})^* on conjugate input
        y = conj(frft_m(conj(x), -a));        % FIXED
        if ~col, y = y.'; end; return;
    end

    % -------- general case: direct discrete FRFT kernel --------
    alpha = a * pi/2;
    sa = sin(alpha);
    ca = cot(alpha);

    if abs(sa) < 1e-12
        y = x; if ~col, y=y.'; end; return;
    end

    % centered discrete indices (improves numerical behavior)
    n = ((0:N-1) - (N-1)/2).';
    k = ((0:N-1) - (N-1)/2);

    % FRFT kernel K(n,k) and apply y(k) = sum_n K(n,k)*x(n)
    % Normalization: scale so energy is roughly preserved.
    % (Exact continuous normalization constants vary; for visualization
    %  and detection this is sufficient and stable.)
    % E = (-1i*pi*ca) * (n.^2 + k.^2) + (1i*2*pi/sa) * (n * k);
    E = (-1i*pi*ca / N) * (n.^2 + k.^2) + (1i*2*pi/(N*sa)) * (n * k);
    K = exp(E);                       % [N x N]
    y = (1/sqrt(abs(sa))) * (1/sqrt(N)) * (K.' * x);   % [N x 1]

    % restore original orientation
    if ~col, y = y.'; end
end
