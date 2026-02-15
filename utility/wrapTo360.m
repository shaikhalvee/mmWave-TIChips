function out = wrapTo360(in)
    out = mod(in, 360);
    out(out < 0) = out(out < 0) + 360;
end