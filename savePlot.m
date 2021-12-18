function savePlot(name, format)
% Saves the current plot.
% name: filename.
% format: file format ('png' or 'eps').

if strcmp(format, 'png')
    print('-dpng', '-r400', name);
else
    print('-depsc2', name);
end

end