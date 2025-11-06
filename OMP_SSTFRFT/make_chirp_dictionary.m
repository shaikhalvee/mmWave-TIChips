% -------------------
% slow-time signal (per range/angle) is well-approximated—over 
% short windows—as a sum of linear-FM components (chirps) 
% from the rotor PMM.
% -------------------
function D = make_chirp_dictionary(L, Ts, f0_grid, mu_grid, win)
% Build short-time chirp atoms:
% g[n] = win[n] * exp(j*2*pi*(f0*n*Ts + 0.5*mu*(n*Ts)^2))
% Returns D: [L, numel(f0_grid)*numel(mu_grid)]

Nf  = numel(f0_grid);
Nm  = numel(mu_grid);
D   = zeros(L, Nf*Nm, 'like', 1+1j);

n   = (0:L-1).';
t   = n*Ts;

col = 1;
for ii = 1:Nf
    f0 = f0_grid(ii);
    for jj = 1:Nm
        mu = mu_grid(jj);
        phi = 2*pi*( f0*t + 0.5*mu*(t.^2) );
        atom = win .* exp(1j*phi);
        D(:,col) = atom;
        col = col + 1;
    end
end
end
