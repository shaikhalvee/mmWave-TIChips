function [R_t, v_t, a_t] = uav_motion_profile_5phase(t, R0_init, v0_init, a_mag, t_cv1, t_accel, t_cv2, t_decel)
%UAV_MOTION_PROFILE_5PHASE
% Piecewise 1D motion (range along a line):
%   Phase 1: 0              <= t < t_cv1                 constant velocity  v = v0_init
%   Phase 2: t_cv1          <= t < t_cv1+t_accel         accelerate        a = +a_mag
%   Phase 3: t_cv1+t_accel  <= t < t_cv1+t_accel+t_cv2   constant velocity  v = v2
%   Phase 4: ...            <= t < ...+t_decel           decelerate to stop (computed a so v_end = 0)
%   Phase 5: t >= t_stop                                  hover             v = 0, a = 0, R = const

    % ---- basic checks ----
    if t_cv1 < 0 || t_accel < 0 || t_cv2 < 0 || t_decel <= 0
        error('Durations must satisfy: t_cv1>=0, t_accel>=0, t_cv2>=0, t_decel>0.');
    end

    % ---- phase boundaries ----
    t1 = t_cv1;
    t2 = t1 + t_accel;
    t3 = t2 + t_cv2;
    t4 = t3 + t_decel;   % stop/hover time

    % ---- precompute boundary states ----
    % End of Phase 1
    R1 = R0_init + v0_init * t1;
    v1 = v0_init;

    % End of Phase 2 (accelerate)
    R2 = R1 + v1 * t_accel + 0.5 * a_mag * t_accel^2;
    v2 = v1 + a_mag * t_accel;

    % End of Phase 3 (constant v2)
    R3 = R2 + v2 * t_cv2;
    v3 = v2;

    % Phase 4 decel acceleration chosen to end at v=0 exactly at t4
    a_decel = -v3 / t_decel;

    % End of Phase 4 position (stop point)
    R_stop = R3 + v3 * t_decel + 0.5 * a_decel * t_decel^2;  % = R3 + 0.5*v3*t_decel

    % ---- evaluate piecewise ----
    if t <= 0
        % before start (hold initial state)
        a_t = 0;
        v_t = v0_init;
        R_t = R0_init;

    elseif t < t1
        % Phase 1: constant velocity
        a_t = 0;
        v_t = v0_init;
        R_t = R0_init + v0_init * t;

    elseif t < t2
        % Phase 2: accelerate +a_mag
        dt = t - t1;
        a_t = a_mag;
        v_t = v1 + a_t * dt;
        R_t = R1 + v1 * dt + 0.5 * a_t * dt.^2;

    elseif t < t3
        % Phase 3: constant velocity at v2
        dt = t - t2;
        a_t = 0;
        v_t = v2;
        R_t = R2 + v2 * dt;

    elseif t < t4
        % Phase 4: decelerate to stop
        dt = t - t3;
        a_t = a_decel;
        v_t = v3 + a_t * dt;
        R_t = R3 + v3 * dt + 0.5 * a_t * dt.^2;

    else
        % Phase 5: hover
        a_t = 0;
        v_t = 0;
        R_t = R_stop;
    end
end
