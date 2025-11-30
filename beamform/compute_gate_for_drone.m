function [gate_idx, gate_center_m, gate_width_m] = compute_gate_for_drone( ...
        to_plot, range_axis, doppler_axis, auto_gate, gate_center_m, gate_width_m, v_exclude)
    % Auto-gate on strongest non-zero-Doppler range; else use manual center/width.
    if auto_gate
        nz = abs(doppler_axis) > v_exclude;
        rp_nz = mean(to_plot(:, nz), 2);
        if ~any(nz)
            rp_nz = mean(to_plot,2);
        end
        [~, pk] = max(rp_nz);
        gate_center_m = range_axis(pk);
    end
    % Ensure >= 1 bin in gate
    if numel(range_axis) > 1
        dr = mean(diff(range_axis));
    else
        dr = 1;
    end
    half_w = max(gate_width_m/2, 0.5*dr);
    mask = (range_axis >= gate_center_m - half_w) & (range_axis <= gate_center_m + half_w);
    if ~any(mask)
        [~, nn] = min(abs(range_axis - gate_center_m));
        mask(nn) = true;
    end
    gate_idx = find(mask);
end