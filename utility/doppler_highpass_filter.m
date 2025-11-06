function out = doppler_highpass_filter(data, filtCoeff)
    out = data;
    [R, ~, RX, ANG] = size(data);
    for r = 1:R
        for rx = 1:RX
            for ang = 1:ANG
                x = squeeze(data(r,:,rx,ang));
                out(r,:,rx,ang) = filter(filtCoeff, 1, x);
            end
        end
    end
end
