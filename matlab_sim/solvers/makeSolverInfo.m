function info = makeSolverInfo(name)
%MAKESOLVERINFO Common solver result structure.

info = struct();
info.name = string(name);
info.success = false;
info.status = "unknown";
info.solveTime = NaN;
info.positionError = inf;
info.orientationError = inf;
info.iterations = NaN;
info.branchId = "";
info.exitFlag = NaN;
info.note = "";
info.raw = [];
end

