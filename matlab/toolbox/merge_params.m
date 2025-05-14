function p = merge_params(defaults, overrides)
%MERGE_PARAMS Overlay the fields of OVERRIDES on DEFAULTS.
%   Unknown field names raise an error so that typos do not pass silently.

p = defaults;
names = fieldnames(overrides);
for k = 1:numel(names)
    if ~isfield(defaults, names{k})
        error('stateSpaceStudio:unknownParameter', 'Unknown parameter "%s".', names{k});
    end
    p.(names{k}) = overrides.(names{k});
end
end
