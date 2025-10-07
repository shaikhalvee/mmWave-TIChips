%-----------------------------------------------------------------------
function err = sensor_advance_frame_BF(devId, params, Tx1_Ph_Deg, Tx2_Ph_Deg, Tx3_Ph_Deg)
% Thin wrapper around TI's RtttNetClientAPI for *one* AWR1243 device.
%   devId ∈ {1,2,3,4} maps to master & 3 slaves.

    import RtttNetClientAPI.*             

    radarId       = [1 2 4 8];            % hard‑coded by TI
    trigSelect    = [1 2 2 2];            % SW trigger for master, HW for slaves

    phase_angle_step = 5.625;

    Tx1_idx = floor(Tx1_Ph_Deg / phase_angle_step);
    Tx2_idx = floor(Tx2_Ph_Deg / phase_angle_step);
    Tx3_idx = floor(Tx3_Ph_Deg / phase_angle_step);

    %-------------------------------------------------- Profile CONFIG
    lua = sprintf(['ar1.ProfileConfig_mult(%d,0,%.3f,%.3f,%.3f,%.3f,0,0,0,0,0,0,' ...
        '%.3f,%.3f,%d,%d,2,1,%d)'], ...
        radarId(devId), params.Start_Freq_GHz, params.Idle_Time_us, ...
        params.Adc_Start_Time_us, params.Ramp_End_Time_us, ...
        params.Slope_MHzperus, params.Tx_Start_Time_us, ...
        params.Samples_per_Chirp, params.Sampling_Rate_ksps, params.Rx_Gain_dB);

    err = RtttNetClient.SendCommand(lua);
    if err ~= 30000, fprintf('Profile Config failed\n'); return; end

    %-------------------------------------------------- Chirp CONFIG
    TXenable = zeros(3,4);
    TXenable(params.Tx_Ant_Arr_BF) = 1;           % map used channels

    for k = 0:params.NumAnglesToSweep-1
        % Chirp config
        lua = sprintf(['ar1.ChirpConfig_mult(%d,%d,%d,0,0,0,0,0,%d,%d,%d)'], ...
            radarId(devId), k, k, ...
            TXenable(1,devId), TXenable(2,devId), TXenable(3,devId));
        
        err = RtttNetClient.SendCommand(lua);
        if err ~= 30000, fprintf('Chirp config failed at chirp %d\n', k); return; end

        % Phase shifter config: use index (not degrees!)
        lua = sprintf(['ar1.SetPerChirpPhaseShifterConfig_mult(%d,%d,%d,%d,%d,%d)'], ...
            radarId(devId), k, k, ...
            Tx1_idx(k+1,devId), Tx2_idx(k+1,devId), Tx3_idx(k+1,devId));
        
        err = RtttNetClient.SendCommand(lua);
        if err ~= 30000, fprintf('Phase config failed at chirp %d\n', k); return; end
    end

    %-------------------------------------------------- Advanced FRAME CONFIG
    p = params; % for brevity
    lua = sprintf(['ar1.AdvanceFrameConfig_mult(%d,%d,1536,0,%d,%d,%d,%d,%d,%d,%d,%d,0,' ...
        '%d,%d,%d,%d,%d,%d,%d,%d,0,0,1,128,8000000,0,1,1,8000000,0,0,1,128,8000000,' ...
        '0,1,1,8000000,%d,%d,0,0,128,256,1,128,256,1,128,1,1,128,1,1)'], ...
        radarId(devId), p.numSubFrames, ...
        p.SF1ChirpStartIdx, p.SF1NumChirps, p.SF1NumLoops, p.SF1BurstPeriodicity, ...
        p.SF1ChirpStartIdxOffset, p.SF1NumBurst, p.SF1NumBurstLoops, p.SF1SubFramePeriodicity, ...
        p.SF1ChirpStartIdx, p.SF1NumChirps, p.SF1NumLoops, p.SF1BurstPeriodicity, ...
        p.SF1ChirpStartIdxOffset, p.SF1NumBurst, p.SF1NumBurstLoops, p.SF1SubFramePeriodicity, ...
        p.Num_Frames, trigSelect(devId));

    err = RtttNetClient.SendCommand(lua);

    if err==30000
        fprintf('[Dev %d] configuration OK\n', devId);
    else
        fprintf('[Dev %d] configuration FAILED\n', devId);
    end
end
