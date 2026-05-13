function summary = run_batch_benchmark(~, ~, sampleCount)
%RUN_BATCH_BENCHMARK 3pit yz/pitch IK benchmark.

if nargin < 3 || isempty(sampleCount)
    sampleCount = 30;
end

summary = run_3pit_benchmark(sampleCount);
end
