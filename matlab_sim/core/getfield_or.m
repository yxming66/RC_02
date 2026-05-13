function value = getfield_or(s, fieldName, fallback)
%GETFIELD_OR Read a field if it exists, otherwise return fallback.

if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = fallback;
end
end

