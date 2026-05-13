function root = armSimRoot()
%ARMSIMROOT Return the matlab_sim root directory.

root = fileparts(fileparts(mfilename("fullpath")));
end

