function y = frft(x, a)
%FRFT Fractional Fourier transform using chirp-FFT-chirp algorithm.
%   y = FRFT(x, a) computes the discrete fractional Fourier transform of
%   signal x with fractional order a (0 corresponds to identity, 1 to the
%   conventional Fourier transform). The implementation follows the
%   chirp-FFT-chirp approach described by Ozaktas et al. and handles the
%   special cases a = 0, 0.5, 1, etc. explicitly for numerical stability.
%
%   Inputs:
%       x - Input vector (column or row).
%       a - Fractional order (real scalar). The transform is periodic with
%           period 4.
%
%   Output:
%       y - Fractional Fourier transform of x with the same orientation
%           (row/column) as the input.
%
%   Note: For a = 1 the transform is equivalent to fftshift(fft(ifftshift(x)))
%   and for a = 0 it equals x. The transform is normalized to be unitary.

    if nargin < 2
        error('frft:NotEnoughInputs', 'Two inputs required: signal and order.');
    end

    wasRow = isrow(x);
    x = x(:);
    n = numel(x);
    if n == 0
        y = x;
        return;
    end

    % Wrap order to [0, 4)
    a = mod(a, 4);

    % Handle the trivial cases explicitly
    if a == 0
        y = x;
    elseif a == 2
        y = flipud(x);
    elseif a == 1
        y = fftshift(fft(ifftshift(x))) / sqrt(n);
    elseif a == 3
        y = fftshift(ifft(ifftshift(x))) * sqrt(n);
    else
        % General case using chirp multiplication and convolution
        alpha = a * pi / 2;
        if abs(sin(alpha)) < 1e-12
            % Near-singular cases fall back to the nearest integer order
            y = frft(x, round(a));
            if ~isfinite(a)
                error('frft:InvalidOrder', 'Order must be a finite scalar.');
            end
        else
            cot_alpha = cot(alpha);
            csc_alpha = 1./sin(alpha);

            % Symmetric index vector centered at zero
            m = (-floor(n/2):ceil(n/2)-1).';

            % Chirp pre- and post-multiplications
            chirp_factor = exp(-1i*pi*cot_alpha*(m.^2)/n);
            x_chirped = x .* chirp_factor;

            % Convolution with another chirp implemented via FFT
            k = (-n+1:n-1).';
            chirp_kernel = exp(1i*pi*csc_alpha*(k.^2)/n);
            L = 2*n - 1;
            X_fft = fft(x_chirped, L);
            H_fft = fft(chirp_kernel, L);
            conv_result = ifft(X_fft .* H_fft);
            conv_result = conv_result(n:(2*n-1));

            y = chirp_factor .* conv_result;
            y = y * exp(-1i*(pi/4)*(1 - a)) / sqrt(abs(sin(alpha)));
        end
    end

    if wasRow
        y = y.';
    end
end
