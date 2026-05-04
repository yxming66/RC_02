function poses = sampleTargetPoses(cfg)
%SAMPLETARGETPOSES Random Cartesian pose samples within configured bounds.

rng(cfg.rngSeed);
n = cfg.sampleCount;
poses = zeros(n, 6);

for i = 1:3
    bounds = cfg.positionBounds(i, :);
    poses(:, i) = bounds(1) + rand(n, 1) * (bounds(2) - bounds(1));
end

for i = 1:3
    bounds = cfg.rpyBounds(i, :);
    poses(:, i + 3) = bounds(1) + rand(n, 1) * (bounds(2) - bounds(1));
end
end

