function [R_t, v_t, a_t] = uav_motion_profile(t, R0_init, v0_init, a_mag, t_accel)
%UAV_MOTION_PROFILE
% Piecewise 1D motion:
%   0          <= t <= t_accel:        accelerate with +a_mag
%   t_accel    <= t <= 2*t_accel:      decelerate with -a_mag
%   t >= 2*t_accel:                    hover (v = 0, a = 0, constant range)

    t_stop = 2 * t_accel;

    if t <= 0
        % before start
        a_t = a_mag;
        R_t = R0_init;
        v_t = v0_init;

    elseif t < t_accel
        % Phase 1: constant +a_mag
        a_t = a_mag;
        R_t = R0_init + v0_init * t + 0.5 * a_t * t.^2;
        v_t = v0_init + a_t * t;

    elseif t < t_stop
        % Phase 2: constant -a_mag
        a_t = -a_mag;

        % State at t_accel
        R1 = R0_init + v0_init * t_accel + 0.5 * a_mag * t_accel^2;
        v1 = v0_init + a_mag * t_accel;

        dt = t - t_accel;
        R_t = R1 + v1 * dt + 0.5 * a_t * dt.^2;
        v_t = v1 + a_t * dt;

    else
        % Phase 3: hover (v=0, a=0)
        a_t = 0;

        % Final position at stop
        R1 = R0_init + v0_init * t_accel + 0.5 * a_mag * t_accel^2;
        v1 = v0_init + a_mag * t_accel;
        dt_stop = t_stop - t_accel;
        R_stop = R1 + v1 * dt_stop + 0.5 * (-a_mag) * dt_stop.^2;

        R_t = R_stop;
        v_t = 0;
    end
end
