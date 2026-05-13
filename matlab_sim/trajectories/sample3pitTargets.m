function poses = sample3pitTargets(cfg)
%SAMPLE3PITTARGETS Random targets for yz/pitch constrained 3pit tests.

rng(cfg.rngSeed);
n = cfg.sampleCount;
poses = repmat(cfg.nominalPose, n, 1);

yBounds = cfg.yBounds;
zBounds = cfg.zBounds;
pitchBounds = cfg.pitchBounds;

poses(:, 2) = yBounds(1) + rand(n, 1) * (yBounds(2) - yBounds(1));
poses(:, 3) = zBounds(1) + rand(n, 1) * (zBounds(2) - zBounds(1));
poses(:, 5) = pitchBounds(1) + rand(n, 1) * (pitchBounds(2) - pitchBounds(1));
end

