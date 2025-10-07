function a = omp_sparse(D, y, K)
% Orthogonal Matching Pursuit: solve y ≈ D a with <= K atoms
% (columns of D assumed L2-normalized)
% D: [L,M], y: [L,1]
% Returns a: [M,1]

[L,M] = size(D);
K = min([K, L, M]);
res = y;
idx_set = zeros(0,1);
a = zeros(M, 1);  % Initialize coefficient vector

for k = 1:K
    % select atom with max correlation
    proj = abs(D' * res);

    if ~isempty(idx_set), proj(idx_set) = 0; end
    [val, idx] = max(proj);
    if val <= 0, break; end

    % [~, idx] = max(proj);
    % idx_set(k) = idx;
    idx_set(end+1,1) = idx;

    % LS update on selected set
    % Ds = D(:, idx_set(1:k));
    Ds = D(:, idx_set);
    % as = Ds \ y;                 % least squares
    as = (Ds' * Ds + 1e-10*eye(size(Ds,2))) \ (Ds' * y);  % stabilized LS

    % update residual
    res = y - Ds*as;

    % stop if residual small
    if norm(res) <= 1e-3 * max(1e-12, norm(y)), break; end

    % stopping if residual very small
    % if norm(res)/max(1e-12, norm(y)) < 1e-3
    %     idx_set = idx_set(1:k);
    %     a(idx_set) = as;
    %     a = a(:);
    %     return;
    % end
end

% fill full coefficient vector
a(idx_set) = as;
% a = a(:);
end
