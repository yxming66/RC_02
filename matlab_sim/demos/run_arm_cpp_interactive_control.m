function app = run_arm_cpp_interactive_control(modelSpec, solverName)
%RUN_ARM_CPP_INTERACTIVE_CONTROL Interactive 3pit yz/pitch validation.
%
% Target controls are dy, dz, and dpitch. The delta can be interpreted in the
% base frame or current tool frame. MATLAB resolves the preview target for the
% UI, while the native C++ exporter resolves the authoritative target, one-shot
% IK result, and bounded control-loop trajectory.

if nargin < 1 || isempty(modelSpec)
    modelSpec = model_3dof_3pit();
end
if nargin < 2 || isempty(solverName)
    solverName = "3pit_hybrid";
end

modelSpec = loadRobotModel(modelSpec);
if modelSpec.dof ~= 3
    error("run_arm_cpp_interactive_control:UnsupportedDof", ...
        "The 3pit interactive simulator expects a 3DOF model.");
end
solverSpec = defaultSolverSpec(solverName);
buildArmCppTrajectoryExporter(false);

state = struct();
state.modelSpec = modelSpec;
state.solverSpec = solverSpec;
state.qCurrent = modelSpec.home;
state.currentPose = tformToPose(fkArm(modelSpec, state.qCurrent));
state.currentJointPositions = computeJointPositions(modelSpec, state.qCurrent);
state.lastDelta = [0, 0, 0];
state.lastTargetPose = state.currentPose;
state.lastQ = state.qCurrent;
state.lastJointPositions = state.currentJointPositions;
state.lastSolver = emptySolverInfo();
state.lastPath = emptyPathInfo();

fig = figure("Name", "3pit yz/pitch interactive control - " + string(modelSpec.name), ...
    "NumberTitle", "off", "MenuBar", "none");
ax = axes("Parent", fig, "Position", [0.34 0.22 0.63 0.72]);
controls = uipanel("Parent", fig, "Title", "3pit target", ...
    "Units", "normalized", "Position", [0.02 0.24 0.29 0.72]);

commandPopup = uicontrol(controls, "Style", "popupmenu", ...
    "String", {"Cartesian path", "Joint path"}, ...
    "Units", "normalized", "Position", [0.08 0.93 0.84 0.05], ...
    "Callback", @commandSpaceChanged);

framePopup = uicontrol(controls, "Style", "popupmenu", ...
    "String", {"Base delta", "Tool delta"}, ...
    "Units", "normalized", "Position", [0.08 0.865 0.84 0.05], ...
    "Callback", @markPending);

solverPopup = uicontrol(controls, "Style", "popupmenu", ...
    "String", {"Analytic", "Numeric", "Analytic + numeric"}, ...
    "Value", solverPopupValue(solverName), ...
    "Units", "normalized", "Position", [0.08 0.80 0.84 0.05], ...
    "Callback", @markPending);

cartNames = ["dy", "dz", "dpitch"];
cartMins = [-0.90, -1.00, -1.60];
cartMaxs = [0.58, 0.08, 1.40];
names = cartNames;
mins = cartMins;
maxs = cartMaxs;
nameLabels = gobjects(1, 3);
sliders = gobjects(1, 3);
valueLabels = gobjects(1, 3);
for i = 1:3
    y = 0.69 - (i - 1) * 0.16;
    nameLabels(i) = uicontrol(controls, "Style", "text", "String", names(i), ...
        "Units", "normalized", "HorizontalAlignment", "left", ...
        "Position", [0.08 y + 0.055 0.28 0.045]);
    valueLabels(i) = uicontrol(controls, "Style", "text", "String", "0", ...
        "Units", "normalized", "HorizontalAlignment", "right", ...
        "Position", [0.60 y + 0.055 0.32 0.045]);
    sliders(i) = uicontrol(controls, "Style", "slider", ...
        "Min", mins(i), "Max", maxs(i), "Value", 0.0, ...
        "Units", "normalized", "Position", [0.08 y 0.84 0.055], ...
        "Callback", @markPending);
end

uicontrol(controls, "Style", "pushbutton", "String", "Resolve + Solve", ...
    "Units", "normalized", "Position", [0.08 0.16 0.84 0.065], ...
    "Callback", @solveTarget);
uicontrol(controls, "Style", "pushbutton", "String", "Apply as current", ...
    "Units", "normalized", "Position", [0.08 0.08 0.84 0.065], ...
    "Callback", @applyCurrent);
uicontrol(controls, "Style", "pushbutton", "String", "Reset target", ...
    "Units", "normalized", "Position", [0.08 0.005 0.84 0.065], ...
    "Callback", @resetTarget);

statusText = uicontrol(fig, "Style", "text", ...
    "Units", "normalized", "HorizontalAlignment", "left", ...
    "Position", [0.02 0.01 0.95 0.19], ...
    "BackgroundColor", get(fig, "Color"), ...
    "FontName", "Consolas", ...
    "String", "");

drawRobot();
updateStatus("idle");

app = struct("figure", fig, "axes", ax, "state", state);

    function delta = currentDelta()
        delta = zeros(1, 3);
        for k = 1:3
            delta(k) = sliders(k).Value;
            valueLabels(k).String = sprintf("%.5g", delta(k));
        end
    end

    function commandSpaceChanged(~, ~)
        resetTargetState();
        drawRobot();
        updateStatus("reset");
    end

    function configureTargetControls()
        names = cartNames;
        mins = cartMins;
        maxs = cartMaxs;
        framePopup.Enable = "on";
        solverPopup.Enable = "on";
        for k = 1:3
            nameLabels(k).String = names(k);
            sliders(k).Min = mins(k);
            sliders(k).Max = maxs(k);
            sliders(k).Value = min(max(sliders(k).Value, mins(k)), maxs(k));
            valueLabels(k).String = sprintf("%.5g", sliders(k).Value);
        end
    end

    function markPending(~, ~)
        delta = currentDelta();
        state.lastDelta = delta;
        state.lastTargetPose = previewTargetPose(state.currentPose, delta, framePopup.Value);
        state.lastQ = state.qCurrent;
        state.lastJointPositions = state.currentJointPositions;
        state.lastSolver = emptySolverInfo();
        state.lastPath = emptyPathInfo();
        drawRobot();
        updateStatus("pending");
    end

    function solveTarget(~, ~)
        delta = currentDelta();
        state.lastDelta = delta;
        input = makeExporterInput(state.currentPose, delta);
        out = runArmCppTrajectoryExporter(input, false);

        frameId = framePopup.Value - 1;
        targetRow = out.frameTargets(out.frameTargets(:, 1) == frameId, :);
        if isempty(targetRow)
            targetRow = out.frameTargets(1, :);
            frameId = targetRow(1, 1);
        end
        state.lastTargetPose = targetRow(1, 2:7);
        [qSolve, solverInfo] = solveWithSelectedMode(out.cppIk, frameId);

        if isJointPath() && solverInfo.success
            moveInput = input;
            moveInput.q1 = qSolve;
            moveInput.controlDuration = -1.0;
            moveOut = runArmCppTrajectoryExporter(moveInput, false);
            solverInfo = attachMoveJResult(solverInfo, moveOut.movej, qSolve);
            state.lastPath = summarizeMoveJPath(moveOut.movej, state.modelSpec);
        else
            state.lastPath = summarizeServoPath(out.servo);
        end
        state.lastSolver = solverInfo;

        if solverInfo.success
            state.lastQ = qSolve;
            state.lastJointPositions = computeJointPositions(state.modelSpec, qSolve);
        end

        drawRobot();
        updateStatus("solved");
    end

    function [q, info] = solveWithSelectedMode(cppIk, frameId)
        row = cppIk(cppIk(:, 1) == frameId, :);
        if isempty(row)
            error("run_arm_cpp_interactive_control:MissingCppIkRow", ...
                "C++ IK output did not contain frame %d.", frameId);
        end
        row = row(1, :);
        q = normalizeJointVector(row(17:19), state.modelSpec.dof);
        info = solverInfoFromCppIkRow(row);
    end

    function input = makeExporterInput(currentPose, delta)
        deltaPose = [0, delta(1), delta(2), 0, delta(3), 0];
        targetPose = previewTargetPose(currentPose, delta, framePopup.Value);
        input = struct();
        input.q0 = state.qCurrent;
        input.q1 = state.qCurrent;
        input.maxVelocity = state.solverSpec.maxVelocity;
        input.maxAcceleration = state.solverSpec.maxAcceleration;
        input.movejDt = state.solverSpec.dt;
        input.cartStartPose = currentPose;
        input.cartGoalPose = targetPose;
        input.cartLimits = struct( ...
            "maxLinearVelocity", state.solverSpec.maxLinearVelocity, ...
            "maxAngularVelocity", state.solverSpec.maxAngularVelocity, ...
            "maxLinearAcceleration", state.solverSpec.maxLinearAcceleration, ...
            "maxAngularAcceleration", state.solverSpec.maxAngularAcceleration);
        input.cartDt = state.solverSpec.dt;
        input.frameDeltaPose = deltaPose;
        input.ikMode = selectedIkModeFromPopup(solverPopup.Value);
        if isJointPath()
            input.controlDuration = -1.0;
        else
            input.controlDuration = 4.0;
        end
        input.maxJointStep = state.solverSpec.controlMaxJointStep;
        input.positionTolerance = state.solverSpec.positionTolerance;
        input.orientationTolerance = state.solverSpec.orientationTolerance;
        input.controlFrame = framePopup.Value - 1;
    end

    function updateStatus(mode)
        solverInfo = state.lastSolver;
        pathInfo = state.lastPath;
        if isJointPath()
            statusFormat = [ ...
                'model=%s | pathMode=%s | frame=%s | solver=%s | path=C++ MoveJ | mode=%s\n', ...
                'delta [dy dz dpitch]=[%.5g %.5g %.5g]\n', ...
                'current [y z pitch]=[%.5g %.5g %.5g] | target [y z pitch]=[%.5g %.5g %.5g]\n', ...
                'solve ok=%d | status=%s | task err pos=%.4g pitch=%.4g | q=[%.5g %.5g %.5g] | movej err=%.4g\n', ...
                'path reached=%d | valid=%d/%d | final pos=%.4g pitch=%.4g | max step=%.4g'];
            statusText.String = sprintf(statusFormat, ...
                char(string(state.modelSpec.name)), ...
                commandPopup.String{commandPopup.Value}, ...
                framePopup.String{framePopup.Value}, ...
                solverPopup.String{solverPopup.Value}, ...
                char(string(mode)), ...
                state.lastDelta(1), state.lastDelta(2), state.lastDelta(3), ...
                state.currentPose(2), state.currentPose(3), state.currentPose(5), ...
                state.lastTargetPose(2), state.lastTargetPose(3), state.lastTargetPose(5), ...
                solverInfo.success, char(string(solverInfo.status)), ...
                solverInfo.positionError, solverInfo.orientationError, ...
                state.lastQ(1), state.lastQ(2), state.lastQ(3), ...
                solverInfo.jointError, ...
                pathInfo.reached, pathInfo.validCount, pathInfo.sampleCount, ...
                pathInfo.finalPositionError, pathInfo.finalOrientationError, ...
                pathInfo.maxJointStep);
            return;
        end
        statusFormat = [ ...
            'model=%s | pathMode=%s | frame=%s | solver=%s | path=C++ control loop | mode=%s\n', ...
            'delta [dy dz dpitch]=[%.5g %.5g %.5g]\n', ...
            'current [y z pitch]=[%.5g %.5g %.5g] | target [y z pitch]=[%.5g %.5g %.5g]\n', ...
            'solve ok=%d | status=%s | task err pos=%.4g pitch=%.4g | full err pos=%.4g ori=%.4g\n', ...
            'path reached=%d | valid=%d/%d | final pos=%.4g pitch=%.4g | max step=%.4g | min singularity=%.4g'];
        statusText.String = sprintf(statusFormat, ...
            char(string(state.modelSpec.name)), ...
            commandPopup.String{commandPopup.Value}, ...
            framePopup.String{framePopup.Value}, ...
            solverPopup.String{solverPopup.Value}, ...
            char(string(mode)), ...
            state.lastDelta(1), state.lastDelta(2), state.lastDelta(3), ...
            state.currentPose(2), state.currentPose(3), state.currentPose(5), ...
            state.lastTargetPose(2), state.lastTargetPose(3), state.lastTargetPose(5), ...
            solverInfo.success, char(string(solverInfo.status)), ...
            solverInfo.positionError, solverInfo.orientationError, ...
            solverInfo.fullPositionError, solverInfo.fullOrientationError, ...
            pathInfo.reached, pathInfo.validCount, pathInfo.sampleCount, ...
            pathInfo.finalPositionError, pathInfo.finalOrientationError, ...
            pathInfo.maxJointStep, pathInfo.minSingularity);
    end

    function drawRobot()
        [az, el] = view(ax);
        hasExistingView = ~isempty(ax.Children);
        show(state.modelSpec.robot, state.lastQ, ...
            "Parent", ax, "PreservePlot", false, "Frames", "off");
        hold(ax, "on");

        drawJointPositions(state.currentJointPositions, [0.2, 0.2, 0.2], "--");
        drawJointPositions(state.lastJointPositions, [0.0, 0.35, 0.95], "-");
        plotFrameAxes(eye(4), 0.06, "base");
        plotFrameAxes(fkArm(state.modelSpec, state.qCurrent), 0.045, "tool");
        plotFrameAxes(poseToTform(state.lastTargetPose), 0.045, "target");

        pCurrent = state.currentPose(1:3);
        pTarget = state.lastTargetPose(1:3);
        plot3(ax, pCurrent(1), pCurrent(2), pCurrent(3), ...
            "go", "MarkerFaceColor", "g");
        plot3(ax, pTarget(1), pTarget(2), pTarget(3), ...
            "rx", "MarkerSize", 9, "LineWidth", 1.6);

        if ~isempty(state.lastPath.targetPos)
            plot3(ax, state.lastPath.targetPos(:, 1), ...
                state.lastPath.targetPos(:, 2), state.lastPath.targetPos(:, 3), ...
                "-", "Color", [0.85, 0.10, 0.06], "LineWidth", 1.8);
        elseif norm(pTarget - pCurrent) > 1e-8
            plot3(ax, [pCurrent(1), pTarget(1)], ...
                [pCurrent(2), pTarget(2)], [pCurrent(3), pTarget(3)], ...
                "r-", "LineWidth", 1.2);
        end
        if ~isempty(state.lastPath.achievedPos)
            plot3(ax, state.lastPath.achievedPos(:, 1), ...
                state.lastPath.achievedPos(:, 2), state.lastPath.achievedPos(:, 3), ...
                "-", "Color", [0.0, 0.45, 0.85], "LineWidth", 1.8, ...
                "Marker", ".", "MarkerSize", 10);
        end

        hold(ax, "off");
        axis(ax, "equal");
        applyWorkspaceView(pCurrent, pTarget);
        if hasExistingView
            view(ax, az, el);
        else
            view(ax, 135, 25);
        end
        axis(ax, "vis3d");
        grid(ax, "on");
        title(ax, "3pit target / solved arm / end-effector trajectory");
        drawnow;
    end

    function applyCurrent(~, ~)
        if ~state.lastSolver.success
            updateStatus("apply skipped");
            return;
        end
        state.qCurrent = state.lastQ;
        state.currentPose = tformToPose(fkArm(state.modelSpec, state.qCurrent));
        state.currentJointPositions = computeJointPositions(state.modelSpec, state.qCurrent);
        resetTargetState();
        drawRobot();
        updateStatus("applied");
    end

    function resetTarget(varargin)
        resetTargetState();
        drawRobot();
        updateStatus("reset");
    end

    function resetTargetState()
        configureTargetControls();
        for k = 1:3
            sliders(k).Value = 0.0;
            valueLabels(k).String = "0";
        end
        state.lastDelta = [0, 0, 0];
        state.lastTargetPose = state.currentPose;
        state.lastQ = state.qCurrent;
        state.lastJointPositions = state.currentJointPositions;
        state.lastSolver = emptySolverInfo();
        state.lastPath = emptyPathInfo();
    end

    function tf = isJointPath()
        tf = commandPopup.Value == 2;
    end

    function info = attachMoveJResult(info, movej, qTarget)
        info.jointError = NaN;
        if isempty(movej)
            info.success = false;
            info.status = string(info.status) + "_empty_movej";
            return;
        end

        qFinal = normalizeJointVector(movej(end, 2:4), state.modelSpec.dof);
        info.jointError = norm(qFinal - qTarget);
        if info.jointError > 1.0e-5 || ~jointWithinLimits(state.modelSpec, qFinal)
            info.success = false;
            info.status = string(info.status) + "_movej_mismatch";
        end
    end
end

function mode = selectedIkModeFromPopup(value)
switch value
    case 1
        mode = 1; % Analytic
    case 2
        mode = 0; % Numeric
    otherwise
        mode = 2; % Analytic seed + numeric refine
end
end

function info = solverInfoFromCppIkRow(row)
info = emptySolverInfo();
info.name = "3pit_cpp";
info.success = row(2) ~= 0;
info.status = statusName(row(3));
info.iterations = row(6);
info.exitFlag = row(3);
info.failStage = row(4);
info.failureReason = row(5);
info.finalError = row(7);
info.fullPositionError = row(8);
info.fullOrientationError = row(9);
info.positionError = row(10);
info.orientationError = row(11);
info.singularity = row(12);
info.limitViolation = row(13);
info.usedAnalytic = row(14) ~= 0;
info.usedNumericRefine = row(15) ~= 0;
info.usedNumericFallback = row(16) ~= 0;
info.raw = row;
end

function name = statusName(value)
names = [ ...
    "success", ...
    "invalid_target", ...
    "invalid_seed", ...
    "invalid_model", ...
    "unreachable", ...
    "constraint_conflict", ...
    "limit_violation", ...
    "singular", ...
    "max_iterations", ...
    "numerical_failure", ...
    "no_convergence", ...
    "unsupported"];
index = round(value) + 1;
if index >= 1 && index <= numel(names)
    name = names(index);
else
    name = "unknown";
end
end

function pose = previewTargetPose(currentPose, delta, frameValue)
pose = currentPose;
if frameValue == 2
    c = cos(currentPose(5));
    s = sin(currentPose(5));
    pose(2) = pose(2) + c * delta(1) - s * delta(2);
    pose(3) = pose(3) + s * delta(1) + c * delta(2);
else
    pose(2) = pose(2) + delta(1);
    pose(3) = pose(3) + delta(2);
end
pose(5) = pose(5) + delta(3);
end

function path = summarizeServoPath(servo)
path = emptyPathInfo();
if isempty(servo)
    return;
end
valid = servo(:, 2) ~= 0;
path.sampleCount = size(servo, 1);
path.validCount = nnz(valid);
path.reached = any(servo(:, 3) ~= 0);
path.targetPos = servo(:, 22:24);
path.achievedPos = servo(:, 28:30);
path.q = servo(:, 16:18);
path.finalPositionError = servo(end, 11);
path.finalOrientationError = servo(end, 12);
path.maxJointStep = max(servo(:, 15));
singularity = servo(valid, 13);
singularity = singularity(isfinite(singularity));
if ~isempty(singularity)
    path.minSingularity = min(singularity);
end
end

function path = summarizeMoveJPath(movej, modelSpec)
path = emptyPathInfo();
if isempty(movej)
    return;
end
q = movej(:, 2:4);
poses = zeros(size(q, 1), 6);
for i = 1:size(q, 1)
    poses(i, :) = tformToPose(fkArm(modelSpec, q(i, :)));
end
path.reached = true;
path.sampleCount = size(q, 1);
path.validCount = size(q, 1);
path.targetPos = [];
path.achievedPos = poses(:, 1:3);
path.q = q;
path.finalPositionError = 0.0;
path.finalOrientationError = 0.0;
if size(q, 1) < 2
    path.maxJointStep = 0.0;
else
    path.maxJointStep = max(vecnorm(diff(q, 1, 1), 2, 2));
end
end

function info = emptySolverInfo()
info = struct( ...
    "success", false, ...
    "status", "idle", ...
    "jointError", NaN, ...
    "positionError", NaN, ...
    "orientationError", NaN, ...
    "fullPositionError", NaN, ...
    "fullOrientationError", NaN);
end

function info = emptyPathInfo()
info = struct( ...
    "reached", false, ...
    "sampleCount", 0, ...
    "validCount", 0, ...
    "targetPos", [], ...
    "achievedPos", [], ...
    "q", [], ...
    "finalPositionError", NaN, ...
    "finalOrientationError", NaN, ...
    "maxJointStep", NaN, ...
    "minSingularity", NaN);
end

function value = solverPopupValue(solverName)
switch lower(string(solverName))
    case {"3pit_analytic", "analytic"}
        value = 1;
    case {"3pit", "numeric"}
        value = 2;
    otherwise
        value = 3;
end
end

function drawJointPositions(modelPoints, color, lineStyle)
ax = gca;
if isempty(modelPoints)
    return;
end
plot3(ax, modelPoints(:, 1), modelPoints(:, 2), modelPoints(:, 3), ...
    "LineStyle", lineStyle, "Color", color, "LineWidth", 1.4, ...
    "Marker", "o", "MarkerFaceColor", color, "MarkerSize", 4);
for idx = 1:size(modelPoints, 1)
    text(ax, modelPoints(idx, 1), modelPoints(idx, 2), modelPoints(idx, 3), ...
        "  J" + string(idx - 1), "Color", color, "FontSize", 8);
end
end

function plotFrameAxes(T, scale, labelText)
ax = gca;
origin = tform2trvec(T);
R = T(1:3, 1:3);
colors = eye(3);
labels = ["x", "y", "z"];
for axisIdx = 1:3
    endpoint = origin + (R(:, axisIdx).' .* scale);
    plot3(ax, [origin(1), endpoint(1)], ...
        [origin(2), endpoint(2)], [origin(3), endpoint(3)], ...
        "-", "Color", colors(axisIdx, :), "LineWidth", 1.5);
    text(ax, endpoint(1), endpoint(2), endpoint(3), ...
        labelText + "_" + labels(axisIdx), ...
        "Color", colors(axisIdx, :), "FontSize", 8);
end
end

function applyWorkspaceView(pCurrent, pTarget)
ax = gca;
points = [0, 0, 0; pCurrent(:).'; pTarget(:).'];
lines = findobj(ax, "Type", "Line");
for i = 1:numel(lines)
    x = lines(i).XData(:);
    y = lines(i).YData(:);
    z = lines(i).ZData(:);
    points = [points; [x, y, z]]; %#ok<AGROW>
end
mins = min(points, [], 1);
maxs = max(points, [], 1);
center = 0.5 .* (mins + maxs);
span = max(max(maxs - mins), 0.35);
half = 0.65 * span;
xlim(ax, center(1) + [-half, half]);
ylim(ax, center(2) + [-half, half]);
zlim(ax, center(3) + [-half, half]);
end

function points = computeJointPositions(modelSpec, q)
q = normalizeJointVector(q, modelSpec.dof);
points = zeros(modelSpec.dof + 1, 3);
points(1, :) = [0, 0, 0];
jointIndex = 1;
for i = 1:numel(modelSpec.robot.Bodies)
    body = modelSpec.robot.Bodies{i};
    if strcmp(body.Joint.Type, "fixed")
        continue;
    end
    jointIndex = jointIndex + 1;
    T = getTransform(modelSpec.robot, q, body.Name, modelSpec.baseName);
    points(jointIndex, :) = tform2trvec(T);
    if jointIndex >= modelSpec.dof + 1
        break;
    end
end
end
