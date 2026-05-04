function sample = sampleCubicBlend(elapsed, duration)
%SAMPLECUBICBLEND Match the C++ cubic blend profile.

if duration <= 1e-6
    sample = struct( ...
        "s", double(elapsed > 0), ...
        "ds", 0.0, ...
        "dds", 0.0, ...
        "finished", elapsed >= 0);
    return;
end

u = min(max(elapsed ./ duration, 0.0), 1.0);
sample = struct();
sample.s = 3.0 .* u.^2 - 2.0 .* u.^3;
sample.ds = 6.0 .* u .* (1.0 - u) ./ duration;
sample.dds = (6.0 - 12.0 .* u) ./ (duration .* duration);
sample.finished = elapsed >= duration;

if sample.finished
    sample.s = 1.0;
    sample.ds = 0.0;
    sample.dds = 0.0;
elseif elapsed <= 0.0
    sample.s = 0.0;
    sample.ds = 0.0;
    sample.dds = 0.0;
end
end

