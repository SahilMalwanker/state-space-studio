function export_figure(fig, opts, name)
%EXPORT_FIGURE Save FIG as <opts.figure_dir>/<name>.png when a folder is configured.

if isfield(opts, 'figure_dir') && strlength(string(opts.figure_dir)) > 0
    exportgraphics(fig, fullfile(string(opts.figure_dir), string(name) + ".png"), ...
        'Resolution', 150);
end
end
