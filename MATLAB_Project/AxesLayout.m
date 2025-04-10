    %% 7) Create Axes for Plots (Radar Echo, Time-Domain and Frequency-Domain)
    axesLeft   = panelWidth + 100;  
    axesWidth  = figWidth  - axesLeft - 100;
    % Zmieniamy podział dostępnej wysokości na 3 wykresy:
    availableHeight = figHeight - 500;  % np. 900-500 = 400
    axesHeight = availableHeight / 3;
    
    % Radar Echo Axes (umieszczony na samej górze)
    handles.hAxesRadar = axes('Parent', hFig, 'Units', 'pixels', ...
        'Position', [axesLeft, 180+2*axesHeight, 0.45*axesWidth, 0.45*axesWidth]);
    title(handles.hAxesRadar, 'Radar Echo');
    xlabel(handles.hAxesRadar, 'Real');
    ylabel(handles.hAxesRadar, 'Imaginary');
    grid(handles.hAxesRadar, 'on');

    % Time-Domain Axes (środkowy)
    handles.hAxesTime = axes('Parent', hFig, 'Units', 'pixels', ...
        'Position', [axesLeft, 100+axesHeight, axesWidth, axesHeight]);
    title(handles.hAxesTime, 'Time Domain');
    xlabel(handles.hAxesTime, 'Sample Index');
    ylabel(handles.hAxesTime, 'ADC Value');
    grid(handles.hAxesTime, 'on');
    set(handles.hAxesTime, 'ButtonDownFcn', @openPlotWindow);

    % Frequency-Domain Axes (dolny)
    handles.hAxesFreq = axes('Parent', hFig, 'Units', 'pixels', ...
        'Position', [axesLeft, 50, axesWidth, axesHeight]);
    title(handles.hAxesFreq, 'Frequency Domain');
    xlabel(handles.hAxesFreq, 'Frequency (Hz)');
    ylabel(handles.hAxesFreq, 'Magnitude');
    grid(handles.hAxesFreq, 'on');
    set(handles.hAxesFreq, 'ButtonDownFcn', @openPlotWindow);