function [R_t, v_t, a_t, info] = uav_motion_profile_outback( ...
    t, R_hold, R_far, R_end, v_max, a_fwd, a_ret, t_hold_start, t_hold_far)
%UAV_MOTION_PROFILE_OUTBACK
% 1D radial motion:
%   Phase A: hold at R_hold for t_hold_start seconds (v=0, a=0)
%   Leg 1:   move R_hold -> R_far, accelerate up to v_max, cruise if needed, decel to stop at R_far
%   Phase B: optional hold at R_far for t_hold_far seconds
%   Leg 2:   move R_far  -> R_end, accelerate up to v_max, cruise if needed, decel to stop at R_end
%   After:   hover at R_end (v=0, a=0)
%
% Sign convention: increasing R = moving away; decreasing R = moving toward.

    if nargin < 9, t_hold_far = 0; end
    if t_hold_start < 0 || t_hold_far < 0
        error('Hold durations must be nonnegative.');
    end
    if v_max <= 0 || a_fwd <= 0 || a_ret <= 0
        error('v_max, a_fwd, a_ret must be positive.');
    end

    % --- Plan leg 1 (R_hold -> R_far) ---
    D1   = abs(R_far - R_hold);
    dir1 = sign(R_far - R_hold);  if dir1 == 0, dir1 = 1; end
    [t1_acc, t1_cruise, v1_peak] = plan_trap_or_tri(D1, v_max, a_fwd);

    % --- Plan leg 2 (R_far -> R_end) ---
    D2   = abs(R_end - R_far);
    dir2 = sign(R_end - R_far);   if dir2 == 0, dir2 = 1; end
    [t2_acc, t2_cruise, v2_peak] = plan_trap_or_tri(D2, v_max, a_ret);

    % --- Time markers ---
    tA0 = 0;
    tA1 = tA0 + t_hold_start;

    tB0 = tA1;
    tB1 = tB0 + t1_acc;
    tB2 = tB1 + t1_cruise;
    tB3 = tB2 + t1_acc;        % symmetric accel/decel durations for each leg

    tC0 = tB3;
    tC1 = tC0 + t_hold_far;

    tD0 = tC1;
    tD1 = tD0 + t2_acc;
    tD2 = tD1 + t2_cruise;
    tD3 = tD2 + t2_acc;

    % --- Precompute boundary states (start at rest after hold) ---
    R_A1 = R_hold;  v_A1 = 0;

    % Leg 1 accel
    a1_acc = dir1 * a_fwd;
    v1     = dir1 * v1_peak;
    R_B1   = R_A1 + v_A1 * t1_acc + 0.5 * a1_acc * t1_acc^2;  % v_A1 = 0
    % Leg 1 cruise
    R_B2   = R_B1 + v1 * t1_cruise;
    % Leg 1 decel
    a1_dec = -dir1 * a_fwd;
    R_B3   = R_B2 + v1 * t1_acc + 0.5 * a1_dec * t1_acc^2;    % ends at R_far
    % Hold far
    R_C1   = R_B3;  v_C1 = 0;

    % Leg 2 accel
    a2_acc = dir2 * a_ret;
    v2     = dir2 * v2_peak;
    R_D1   = R_C1 + v_C1 * t2_acc + 0.5 * a2_acc * t2_acc^2;  % v_C1 = 0
    % Leg 2 cruise
    R_D2   = R_D1 + v2 * t2_cruise;
    % Leg 2 decel
    a2_dec = -dir2 * a_ret;
    R_D3   = R_D2 + v2 * t2_acc + 0.5 * a2_dec * t2_acc^2;    % ends at R_end

    % --- Output diagnostic info ---
    info = struct();
    info.t_hold_start = t_hold_start;
    info.t_hold_far   = t_hold_far;
    info.t_markers    = [tA1, tB1, tB2, tB3, tC1, tD1, tD2, tD3];
    info.R_markers    = [R_A1, R_B1, R_B2, R_B3, R_C1, R_D1, R_D2, R_D3];
    info.v_peaks      = [v1, v2];
    info.total_time   = tD3;

    % --- Piecewise evaluation ---
    if t <= 0
        R_t = R_hold; v_t = 0; a_t = 0;

    elseif t < tA1
        % Phase A: hold at start
        R_t = R_hold; v_t = 0; a_t = 0;

    elseif t < tB1
        % Leg 1 accel
        dt  = t - tB0;
        a_t = a1_acc;
        v_t = v_A1 + a_t * dt;
        R_t = R_A1 + v_A1 * dt + 0.5 * a_t * dt^2;

    elseif t < tB2
        % Leg 1 cruise
        dt  = t - tB1;
        a_t = 0;
        v_t = v1;
        R_t = R_B1 + v1 * dt;

    elseif t < tB3
        % Leg 1 decel to stop
        dt  = t - tB2;
        a_t = a1_dec;
        v_t = v1 + a_t * dt;
        R_t = R_B2 + v1 * dt + 0.5 * a_t * dt^2;

    elseif t < tC1
        % Phase B: optional hold at far point
        R_t = R_B3; v_t = 0; a_t = 0;

    elseif t < tD1
        % Leg 2 accel
        dt  = t - tD0;
        a_t = a2_acc;
        v_t = v_C1 + a_t * dt;
        R_t = R_C1 + v_C1 * dt + 0.5 * a_t * dt^2;

    elseif t < tD2
        % Leg 2 cruise
        dt  = t - tD1;
        a_t = 0;
        v_t = v2;
        R_t = R_D1 + v2 * dt;

    elseif t < tD3
        % Leg 2 decel to stop
        dt  = t - tD2;
        a_t = a2_dec;
        v_t = v2 + a_t * dt;
        R_t = R_D2 + v2 * dt + 0.5 * a_t * dt^2;

    else
        % Final hover at R_end
        R_t = R_D3; v_t = 0; a_t = 0;
    end
end

% ---- helper: trapezoid if reachable, else triangular ----
function [t_acc, t_cruise, v_peak] = plan_trap_or_tri(D, v_max, a_mag)
    if D <= 0
        t_acc = 0; t_cruise = 0; v_peak = 0; return;
    end
    D_min_for_vmax = v_max^2 / a_mag;  % accel+decel distance to hit v_max
    if D >= D_min_for_vmax
        v_peak  = v_max;
        t_acc   = v_peak / a_mag;
        D_acc   = v_peak^2 / (2*a_mag);
        t_cruise = (D - 2*D_acc) / v_peak;
    else
        % triangular: never reaches v_max
        v_peak  = sqrt(D * a_mag);
        t_acc   = v_peak / a_mag;
        t_cruise = 0;
    end
end
