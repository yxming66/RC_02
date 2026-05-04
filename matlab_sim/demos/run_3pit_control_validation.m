function results = run_3pit_control_validation(doAnimate)
%RUN_3PIT_CONTROL_VALIDATION Validate 3pit control-loop behavior with C++ code.

if nargin < 1 || isempty(doAnimate)
    doAnimate = true;
end

modelSpec = loadRobotModel(model_3dof_3pit());
solverSpec = defaultSolverSpec("3pit");

fprintf("3pit control validation model: %s\n", modelSpec.name);
fprintf("C++ path: User/robotics/arm numeric constrained IK + MoveJ + Cartesian trajectory + JointServo + safety\n");

qCurrent = modelSpec.controlHome;
scenarios = makeScenarios(modelSpec, qCurrent);
scenarioResults = repmat(struct(), 1, numel(scenarios));

for i = 1:numel(scenarios)
    scenario = scenarios(i);
    input = makeScenarioInput(modelSpec, solverSpec, qCurrent, scenario);
    out = runArmCppTrajectoryExporter(input, i == 1);
    summary = summarizeServo(out.servo, modelSpec, solverSpec);
    scenarioResults(i).name = scenario.name;
    scenarioResults(i).input = input;
    scenarioResults(i).out = out;
    scenarioResults(i).summary = summary;

    fprintf("[%s] reached=%d valid=%d/%d ikFail=%d safetyFail=%d finalTaskPos=%.4g finalTaskOri=%.4g maxStep=%.4g\n", ...
        scenario.name, summary.reached, summary.validCount, summary.sampleCount, ...
        summary.ikFailureCount, summary.safetyFailureCount, ...
        summary.finalTaskPositionError, summary.finalTaskOrientationError, ...
        summary.maxJointStep);

    if summary.reached
        qCurrent = summary.finalQ;
    end
end

results = struct();
results.modelSpec = modelSpec;
results.solverSpec = solverSpec;
results.scenarios = scenarioResults;
results.allReached = all(arrayfun(@(s) s.summary.reached, scenarioResults));
results.totalIkFailures = sum(arrayfun(@(s) s.summary.ikFailureCount, scenarioResults));
results.totalSafetyFailures = sum(arrayfun(@(s) s.summary.safetyFailureCount, scenarioResults));

fprintf("Overall: reached=%d/%d, ik failures=%d, safety failures=%d\n", ...
    nnz(arrayfun(@(s) s.summary.reached, scenarioResults)), ...
    numel(scenarioResults), results.totalIkFailures, results.totalSafetyFailures);

plotScenarioSummaries(scenarioResults);
if doAnimate
    qPath = vertcatScenarioQ(scenarioResults);
    if ~isempty(qPath)
        animateArmTrajectory(modelSpec, qPath, "3pit C++ control-loop q path", 0.01);
    end
end
end

function scenarios = makeScenarios(modelSpec, q0)
startPose = tformToPose(fkArm(modelSpec, q0));

scenarios = repmat(struct( ...
    "name", "", ...
    "mode", "delta", ...
    "frame", 0, ...
    "deltaPose", zeros(1, 6), ...
    "targetPose", startPose), 1, 3);

scenarios(1).name = "base_delta_yz_pitch";
scenarios(1).mode = "delta";
scenarios(1).frame = 0;
scenarios(1).deltaPose = [0.0, 0.015, 0.015, 0.0, 0.02, 0.0];
scenarios(1).targetPose = tformToPose(poseToTform(scenarios(1).deltaPose) * ...
    fkArm(modelSpec, q0));

scenarios(2).name = "tool_delta_yz_pitch";
scenarios(2).mode = "delta";
scenarios(2).frame = 1;
scenarios(2).deltaPose = [0.0, 0.012, -0.010, 0.0, -0.015, 0.0];
scenarios(2).targetPose = startPose;

scenarios(3).name = "absolute_yz_pitch";
scenarios(3).mode = "absolute";
scenarios(3).frame = 0;
scenarios(3).deltaPose = zeros(1, 6);
scenarios(3).targetPose = [0.0, -0.012, 0.012, 0.0, -0.015, 0.0];
end

function input = makeScenarioInput(modelSpec, solverSpec, qCurrent, scenario)
currentPose = tformToPose(fkArm(modelSpec, qCurrent));
qMoveJ = clampToLimits(modelSpec, qCurrent + [0.18, -0.14, 0.12]);

input = struct();
input.q0 = qCurrent;
input.q1 = qMoveJ;
input.maxVelocity = solverSpec.maxVelocity;
input.maxAcceleration = solverSpec.maxAcceleration;
input.movejDt = solverSpec.dt;
input.cartStartPose = currentPose;
if scenario.mode == "absolute"
    input.cartGoalPose = currentPose + scenario.targetPose;
else
    input.cartGoalPose = scenario.targetPose;
end
input.cartLimits = struct( ...
    "maxLinearVelocity", solverSpec.maxLinearVelocity, ...
    "maxAngularVelocity", solverSpec.maxAngularVelocity, ...
    "maxLinearAcceleration", solverSpec.maxLinearAcceleration, ...
    "maxAngularAcceleration", solverSpec.maxAngularAcceleration);
input.cartDt = solverSpec.dt;
input.frameDeltaPose = scenario.deltaPose;
input.ikMode = solverSpec.ikMode;
input.controlDuration = 4.0;
input.maxJointStep = solverSpec.controlMaxJointStep;
input.positionTolerance = solverSpec.positionTolerance;
input.orientationTolerance = solverSpec.orientationTolerance;
input.controlFrame = scenario.frame;
end

function summary = summarizeServo(servo, modelSpec, solverSpec)
q = servo(:, 16:18);
qd = servo(:, 19:21);
valid = servo(:, 2) ~= 0;
reached = servo(:, 3) ~= 0;
ikOk = servo(:, 4) ~= 0;
safetyOk = servo(:, 5) ~= 0;

summary = struct();
summary.sampleCount = size(servo, 1);
summary.validCount = nnz(valid);
summary.reached = any(reached);
summary.ikFailureCount = nnz(~ikOk);
summary.safetyFailureCount = nnz(~safetyOk);
summary.finalQ = normalizeJointVector(q(end, :), modelSpec.dof);
summary.finalTaskPositionError = servo(end, 11);
summary.finalTaskOrientationError = servo(end, 12);
summary.maxTaskPositionError = max(servo(:, 11));
summary.maxTaskOrientationError = max(servo(:, 12));
summary.maxJointStep = max(servo(:, 15));
velocityMargin = abs(qd) - repmat(solverSpec.maxVelocity, size(qd, 1), 1);
summary.maxVelocityViolation = max([0; velocityMargin(:)]);
summary.withinLimits = all(arrayfun(@(i) jointWithinLimits(modelSpec, q(i, :)), 1:size(q, 1)));
summary.statusCodes = servo(:, 6);
summary.failureReasons = servo(:, 8);
end

function q = vertcatScenarioQ(scenarioResults)
q = [];
for i = 1:numel(scenarioResults)
    if isfield(scenarioResults(i), "out") && isfield(scenarioResults(i).out, "servo")
        q = [q; scenarioResults(i).out.servo(:, 16:18)]; %#ok<AGROW>
    end
end
end

function plotScenarioSummaries(scenarioResults)
figure("Name", "3pit C++ control validation");
tiledlayout(3, 1);

nexttile;
hold on;
for i = 1:numel(scenarioResults)
    servo = scenarioResults(i).out.servo;
    plot(servo(:, 1), servo(:, 16:18), "LineWidth", 1.0);
end
hold off;
grid on;
ylabel("q (rad)");
title("3pit joint command path");

nexttile;
hold on;
for i = 1:numel(scenarioResults)
    servo = scenarioResults(i).out.servo;
    semilogy(servo(:, 1), max(servo(:, 11), eps), "LineWidth", 1.0);
end
hold off;
grid on;
ylabel("task pos err");

nexttile;
hold on;
for i = 1:numel(scenarioResults)
    servo = scenarioResults(i).out.servo;
    semilogy(servo(:, 1), max(servo(:, 12), eps), "LineWidth", 1.0);
end
hold off;
grid on;
xlabel("time (s)");
ylabel("task pitch err");
legend(string(arrayfun(@(s) s.name, scenarioResults, "UniformOutput", false)), ...
    "Interpreter", "none", "Location", "best");
end

function q = clampToLimits(modelSpec, q)
limits = modelSpec.jointLimits;
q = normalizeJointVector(q, modelSpec.dof);
q = min(max(q, limits(:, 1).'), limits(:, 2).');
end
