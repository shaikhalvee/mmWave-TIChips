function a = omp_sparse(D, y, K)
% Orthogonal Matching Pursuit: solve y ≈ D a with <= K atoms
% (columns of D assumed L2-normalized)
% D: [L,M], y: [L,1]
% Returns a: [M,1]

[L,M] = size(D);
K = min([K, L, M]);
res = y;
idx_set = zeros(K,1);
a = zeros(M, 1);  % Initialize coefficient vector

for k = 1:K
    % select atom with max correlation
    proj = abs(D' * res);
    
    [~, idx] = max(proj);
    idx_set(k) = idx;

    % LS update on selected set
    Ds = D(:, idx_set(1:k));
    as = Ds \ y;                 % least squares

    % update residual
    res = y - Ds*as;

    % stopping if residual very small
    if norm(res)/max(1e-12, norm(y)) < 1e-3
        idx_set = idx_set(1:k);
        a(idx_set) = as;
        a = a(:);
        return;
    end
end

% fill full coefficient vector
a(idx_set) = as;
a = a(:);
end
