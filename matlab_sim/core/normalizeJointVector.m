function q = normalizeJointVector(q, dof)
%NORMALIZEJOINTVECTOR Return q as a row vector of length dof.

q = double(q(:).');
if numel(q) ~= dof
    error("normalizeJointVector:InvalidSize", ...
        "Expected %d joints, got %d.", dof, numel(q));
end
end

