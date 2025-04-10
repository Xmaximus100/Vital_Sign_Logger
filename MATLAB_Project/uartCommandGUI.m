function uartCommandGUI
    %% 1) Create the Main Figure
    figWidth = 1400;
    figHeight = 900;
    screenSize = get(0, 'ScreenSize');
    leftPos = (screenSize(3) - figWidth) / 2;
    bottomPos = (screenSize(4) - figHeight) / 2;

    hFig = figure('Name', 'UART Command GUI', 'NumberTitle', 'off', ...
                  'Position', [leftPos, bottomPos, figWidth, figHeight], ...
                  'MenuBar', 'none', 'ToolBar', 'none');

    %% 2) Create a Panel for Commands on the Left
    panelWidth  = 300;
    panelHeight = figHeight - 20;  % Slight padding
    hPanel = uipanel('Parent', hFig, ...
                     'Title', 'Commands', ...
                     'Units', 'pixels', ...
                     'Position', [10, 10, panelWidth, panelHeight]);

    %% 3) Add Popup Menus for COM Port and Baud Rate at the Top of the Panel
    ports = serialportlist("available");
    if isempty(ports)
        ports = {'COM1'};
    end
    baudList = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];

    uicontrol('Parent', hPanel, 'Style', 'text', 'String', 'Port:', ...
              'Position', [10, panelHeight-50, 40, 20], ...
              'HorizontalAlignment', 'left');
    handles.hPortPopup = uicontrol('Parent', hPanel, 'Style', 'popupmenu', ...
        'String', ports, 'Value', 1, ...
        'Position', [50, panelHeight-54, 100, 25], ...
        'Callback', @(src,evt) onPortSelected(src,evt));

    uicontrol('Parent', hPanel, 'Style', 'text', 'String', 'Baud:', ...
              'Position', [160, panelHeight-50, 40, 20], ...
              'HorizontalAlignment', 'left');
    handles.hBaudPopup = uicontrol('Parent', hPanel, 'Style', 'popupmenu', ...
        'String', arrayfun(@num2str, baudList, 'UniformOutput', false), ...
        'Value', find(baudList==460800,1), ...
        'Position', [200, panelHeight-54, 80, 25], ...
        'Callback', @(src,evt) onBaudSelected(src,evt));

    %% 4) Status Indicator and Stop Button
    uicontrol('Parent', hPanel, 'Style', 'text', 'String', 'Status:', ...
              'Position', [10, panelHeight-90, 50, 20], ...
              'HorizontalAlignment', 'left');
    handles.hStatusIndicator = uipanel('Parent', hPanel, 'Units', 'pixels', ...
              'Position', [65, panelHeight-90, 30, 30], ...
              'BackgroundColor', [0 1 0]);
    handles.hStopButton = uicontrol('Parent', hPanel, 'Style', 'pushbutton', ...
              'String', 'Stop', 'Position', [110, panelHeight-90, 80, 30], ...
              'Callback', @stopButtonCallback);
    handles.stopRequested = false;
    handles.rangeValue = 10;  % nieużywane przy skalowaniu

    %% 5) Define Command Settings
    cmdList = { ...
        struct('name','SETUP',    'hasValue', false), ...
        struct('name','RUN',      'hasValue', false), ...
        struct('name','READRANGE','hasValue', false), ...
        struct('name','FREQOut',  'hasValue', true,  'range',[100 15000],   'default',12000), ...
        struct('name','FREQIn',   'hasValue', true,  'range',[5 200],       'default',100), ...
        struct('name','POW',      'hasValue', true,  'range',[0 3],         'default',3), ...
        struct('name','CURR',     'hasValue', true,  'range',[310 5000],    'default',500), ...
        struct('name','MUXOUT',   'hasValue', true,  'range',[0 6],         'default',5), ...
        struct('name','EN',       'hasValue', true,  'range',[0 1],         'default',1), ...
        struct('name','LED',      'hasValue', true,  'range',[0 1],         'default',0), ...
        struct('name','RANGE',    'hasValue', true,  'range',[0 2],         'default',2), ...
        struct('name','CONMODE',  'hasValue', true,  'range',[0 1],         'default',0), ...
        struct('name','READRAW',  'hasValue', true,  'range',[1 16777216],  'default',100000) ...
    };
    nCmd = numel(cmdList);
    rowHeight = 35;
    spacing   = 5;
    labelWidth = 60;
    editWidth  = 80;
    btnWidth   = 50;
    startY = panelHeight - 150;
    handles.cmdControls = struct();
    for i = 1:nCmd
        yPos = startY - (i-1)*(rowHeight + spacing);
        uicontrol('Parent', hPanel, 'Style', 'text', ...
            'String', cmdList{i}.name, 'Position', [10, yPos+5, labelWidth, rowHeight-10], ...
            'HorizontalAlignment', 'left');
        if cmdList{i}.hasValue
            handles.cmdControls.(cmdList{i}.name).edit = uicontrol('Parent', hPanel, ...
                'Style', 'edit', 'String', num2str(cmdList{i}.default), ...
                'Position', [10+labelWidth+5, yPos+3, editWidth, rowHeight-5]);
        else
            handles.cmdControls.(cmdList{i}.name).edit = [];
        end
        handles.cmdControls.(cmdList{i}.name).button = uicontrol('Parent', hPanel, ...
            'Style', 'pushbutton', 'String', 'Send', ...
            'Position', [10+labelWidth+editWidth+10, yPos+3, btnWidth, rowHeight-5], ...
            'Callback', @(src,event) sendCommandCallback(cmdList{i}));
    end

    %% 6) "Append Plot" Checkbox, "Live Data" Checkbox and Response Box
    checkboxY = 170;
    handles.hAppendCheckbox = uicontrol('Parent', hPanel, 'Style', 'checkbox', ...
        'String', 'Append Plot', 'Position', [10, checkboxY, 120, 20], ...
        'Value', 0);
    handles.hLiveDataCheckbox = uicontrol('Parent', hPanel, 'Style', 'checkbox', ...
        'String', 'Live Data', 'Position', [120, checkboxY, 100, 20], ...
        'Value', 0);
    handles.hSaveDataCheckbox = uicontrol('Parent', hPanel, 'Style', 'checkbox', ...
        'String', 'Save Data', 'Position', [210, checkboxY, 100, 20], ...
        'Value', 0);
    textBoxHeight = 140;
    handles.hResponseBox = uicontrol('Parent', hPanel, 'Style', 'edit', ...
        'Max', 10, 'Min', 1, 'HorizontalAlignment', 'left', ...
        'Position', [10, 10, panelWidth - 20, textBoxHeight], ...
        'String', '', 'Enable', 'inactive');

    %% 7) Create Axes for Plots (Radar Echo, Time-Domain, Frequency-Domain)
    axesLeft   = panelWidth + 100;
    axesWidth  = figWidth  - axesLeft - 100;
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

    handles.comPort = ports{1};
    handles.baudRate = baudList(find(baudList==460800,1));
    guidata(hFig, handles);

    %-------------------------------------------------------
    % CALLBACKS & FUNCTIONS
    %-------------------------------------------------------
    function onPortSelected(src, ~)
        handles = guidata(hFig);
        val = get(src, 'Value');
        handles.comPort = ports{val};
        guidata(hFig, handles);
    end

    function onBaudSelected(src, ~)
        handles = guidata(hFig);
        val = get(src, 'Value');
        handles.baudRate = baudList(val);
        guidata(hFig, handles);
    end

    function stopButtonCallback(~, ~)
        handles = guidata(hFig);
        handles.stopRequested = true;
        set(handles.hResponseBox, 'String', 'Stop requested.');
        guidata(hFig, handles);
    end

    function sendCommandCallback(cmdStruct)
        handles = guidata(hFig);
        cmdName = cmdStruct.name;
        if cmdStruct.hasValue
            valueStr = get(handles.cmdControls.(cmdName).edit, 'String');
            value = str2double(valueStr);
            if isnan(value) || (mod(value,1) ~= 0)
                errordlg([cmdName ' requires an integer value.'], 'Invalid Input');
                return;
            end
            if value < cmdStruct.range(1) || value > cmdStruct.range(2)
                errordlg(sprintf('%s must be between %d and %d.', ...
                                 cmdName, cmdStruct.range(1), cmdStruct.range(2)), ...
                         'Invalid Range');
                return;
            end
        end

        if strcmp(cmdName, 'READRAW')
            handles.stopRequested = false;
            set(handles.hStatusIndicator, 'BackgroundColor', [1 0 0]); % busy (red)
            guidata(hFig, handles);
            numSamples = value;
            % Uruchamiamy pobieranie danych i aktualizację wykresów "na żywo"
            readRawData(numSamples, handles.comPort, handles.baudRate);
            set(handles.hStatusIndicator, 'BackgroundColor', [0 1 0]); % free (green)
            return;
        end

        if cmdStruct.hasValue
            commandStr = sprintf('%s=%d', cmdName, value);
        else
            commandStr = cmdName;
        end

        response = sendUARTCommand(commandStr, handles.comPort, handles.baudRate);
        if ~strcmp(response, '') && strcmp(cmdName, 'RANGE')
            ADCranges = [2.5, 5, 10];
            handles.rangeValue = ADCranges(value+1);
        end
        set(handles.hResponseBox, 'String', response);
    end

    function response = sendUARTCommand(commandStr, comPort, baudRate)
        try
            s = serialport(comPort, baudRate);
            configureTerminator(s, "LF");
            s.Timeout = 30;
            flush(s);
            writeline(s, commandStr);
            response = readline(s);
            clear s;
        catch ME
            response = ['Error: ' ME.message];
        end
    end

    % Wersja readRawData, która w pętli odczytuje ramki, aktualizuje wykresy
    % Time Domain i Radar Echo "na żywo". Po zakończeniu pętli rysowany jest Frequency Domain.
    function [sampleIndex, ch1, ch2, ch3, ch4] = readRawData(numSamples, comPort, baudRate)
        livedata = get(handles.hLiveDataCheckbox, 'Value');
        savedata = get(handles.hSaveDataCheckbox, 'Value');
        stop = 0;
        s = serialport(comPort, baudRate);
        configureTerminator(s, "LF");
        s.Timeout = 30;
        flush(s);
        cmdStr = sprintf("READRAW=%d", numSamples);
        writeline(s, cmdStr);
        
        sampleIndex = [];
        ch1 = [];
        ch2 = [];
        ch3 = [];
        ch4 = [];
        rawMessages = cell(numSamples,1);
        
        for i = 1:numSamples
            handles = guidata(hFig);
            if handles.stopRequested
                stop = 1;
                warning('Stop requested: aborting data acquisition.');
                break;
            end
            try
                rawMessages{i} = read(s, 11, "uint8");
            catch ME
                warning('Failed to read sample %d: %s', i, ME.message);
                break;
            end
            
            if numel(rawMessages{i}) ~= 11
                warning('Frame %d is too short.', i);
                continue;
            end
            if livedata
                frame = rawMessages{i};
                idx = uint32(frame(3))*65536 + uint32(frame(2))*256 + uint32(frame(1));
                sampleIndex = [sampleIndex, idx];
                ch1_val = uint16(frame(5))*256 + uint16(frame(4));
                ch2_val = uint16(frame(7))*256 + uint16(frame(6));
                ch3_val = uint16(frame(9))*256 + uint16(frame(8));
                ch4_val = uint16(frame(11))*256 + uint16(frame(10));
                ch1 = [ch1, ch1_val];
                ch2 = [ch2, ch2_val];
                ch3 = [ch3, ch3_val];
                ch4 = [ch4, ch4_val];
                % Aktualizacja wykresów na żywo (Time Domain i Radar Echo)
                updateLivePlots(sampleIndex, ch1, ch2, ch3, ch4);
            end
            
            set(handles.hResponseBox, 'String', num2str(rawMessages{i}));
            
            
            %pause(0.01);
        end
        clear s;
        % Po zakończeniu odczytu, rysujemy Frequency Domain, jeśli jest wystarczająca liczba próbek
        if ~livedata
            numSamplesActual = numel(rawMessages);
            sampleIndex = zeros(1, numSamplesActual);
            ch1 = zeros(1, numSamplesActual);
            ch2 = zeros(1, numSamplesActual);
            ch3 = zeros(1, numSamplesActual);
            ch4 = zeros(1, numSamplesActual);
    
            if stop == 1
                return
            end
            
            for i = 1:numSamplesActual
                frame = rawMessages{i};
                if numel(frame) ~= 11
                    warning('Frame %d is too short.', i);
                    continue;
                end
                sampleIndex(i) = uint32(frame(3))*65536 + uint32(frame(2))*256 + uint32(frame(1));
                ch1(i) = uint16(frame(5))*256 + uint16(frame(4));
                ch2(i) = uint16(frame(7))*256 + uint16(frame(6));
                ch3(i) = uint16(frame(9))*256 + uint16(frame(8));
                ch4(i) = uint16(frame(11))*256 + uint16(frame(10));
            end
        end
        Icurrent = updateLivePlots(sampleIndex, ch1, ch2, ch3, ch4);
        updateFrequencyDomainPlot(sampleIndex, ch1, ch2, ch3, ch4);
        if savedata 
            index = 0;
            folderName = string(datetime('now', 'Format', 'dd_MM_yyyy'));
            
            % Check if the folder exists; if not, create it.
            if ~exist(folderName, 'dir')
                mkdir(folderName);
            end
            
            % Create an initial file name with full path (including .mat extension)
            currentFileName = fullfile(folderName, sprintf('testy_%d.mat', index));
            
            % Loop until a filename is found that doesn't exist in the folder.
            while exist(currentFileName, 'file')
                index = index + 1;
                currentFileName = fullfile(folderName, sprintf('testy_%d.mat', index));
            end
            
            % Save the variables to the new file
            save(currentFileName, 'ch1', 'ch2', 'ch3', 'ch4', 'Icurrent', 'sampleIndex');
        end
    end

    % Aktualizacja wykresów, które mają być odświeżane "na żywo"
    function Icurrent = updateLivePlots(sampleIndex, ch1, ch2, ch3, ch4)
        handles = guidata(hFig);
        appendData = get(handles.hAppendCheckbox, 'Value');
        
        if numel(ch1) < 2
            % Jeśli nie ma wystarczającej liczby próbek, pomijamy aktualizację
            return;
        end
        %% Time-Domain Plot
        axTime = handles.hAxesTime;
        if ~appendData, cla(axTime); end
        hold(axTime, 'on');
        plot(axTime, sampleIndex, ch1, 'r');
        plot(axTime, sampleIndex, ch2, 'g');
        plot(axTime, sampleIndex, ch3, 'b');
        plot(axTime, sampleIndex, ch4, 'k');
        hold(axTime, 'off');
        D = cat(4,ch1,ch2,ch3,ch4);
        ylim(axTime, [min(D(:)), max(D(:))]);
        legend(axTime, 'CH1','CH2','CH3','CH4');
        
        %% Radar Echo Plot Update
        % Obliczamy echo radarowe dla bieżących danych (przyjmujemy, że funkcja radar_echo przetwarza pełne próbki)
        Icurrent = radar_echo(ch1, ch2, ch3, ch4);
        axRadar = handles.hAxesRadar;
        cla(axRadar);
        hold(axRadar, 'on');
        % Rysujemy pełną trajektorię echa – dla uproszczenia używamy wszystkich wyliczonych punktów
        plot(axRadar, real(Icurrent), imag(Icurrent), 'o-');
        % Wyróżniamy bieżący punkt
        currentPoint = Icurrent(end);
        plot(axRadar, real(currentPoint), imag(currentPoint), 'o', 'MarkerSize', 8, 'LineWidth', 2);
        hold(axRadar, 'off');
        
        drawnow;
    end

    % Funkcja aktualizująca Frequency Domain, wykonywana po zakończeniu odczytu
    function updateFrequencyDomainPlot(sampleIndex, ch1, ch2, ch3, ch4)
        if numel(ch1) < 2
            % Jeśli nie ma wystarczającej liczby próbek, pomijamy aktualizację
            return;
        end
        handles = guidata(hFig);
        axFreq = handles.hAxesFreq;
        %Icurrent = radar_echo(ch1, ch2, ch3, ch4);
        cla(axFreq);
        hold(axFreq, 'on');
        fch1 = fft(ch1);
        fch2 = fft(ch2);
        fch3 = fft(ch3);
        fch4 = fft(ch4);
        %fch5 = fft(Icurrent);
        fsampling = 1000;
        f = linspace(0, fsampling, length(fch1));
        %subplot(2,1,1);
        plot(axFreq, f, abs(fch1), 'r');
        plot(axFreq, f, abs(fch2), 'g');
        plot(axFreq, f, abs(fch3), 'b');
        plot(axFreq, f, abs(fch4), 'k');
        %subplot(2,1,2);
        %plot(axFreq, f, abs(fch5), 'b');
        % Ustawienie osi Y tylko, gdy liczba próbek pozwala na wykonanie operacji
        hold(axFreq, 'off');
        legend(axFreq, 'CH1','CH2','CH3','CH4');
        drawnow;
    end

    function openPlotWindow(src, ~)
        hNewFig = figure('Name', 'Extracted Plot', 'NumberTitle', 'off');
        newAx = copyobj(src, hNewFig);
        set(newAx, 'Units', 'normalized', 'Position', [0.13 0.11 0.775 0.815]);
    end

    function I = radar_echo(ch1, ch2, ch3, ch4)
        ch1 = double(ch1);
        ch2 = double(ch2);
        ch3 = double(ch3);
        ch4 = double(ch4);

        P4 = (ch3 - 14970.8) / 369.597;
        P3 = (ch2 - 14970.8) / 369.597;
        P2 = (ch1 - 14970.8) / 369.597;
        Pref = (ch4 - 14970.8) / 369.597;
        P4norm = P4 - Pref;
        P3norm = P3 - Pref;
        P2norm = P2 - Pref;
        P4norm = 10.^(P4norm/10);
        P3norm = 10.^(P3norm/10);
        P2norm = 10.^(P2norm/10);
        I = -P2norm + 0.5*P3norm + 0.5*P4norm + 1i*sqrt(3)/2*(-P3norm + P4norm);
    end

    hFig.CloseRequestFcn = @(src, event) cleanupAndClose(src);
    function cleanupAndClose(figHandle)
        timers = timerfindall;
        if ~isempty(timers)
            stop(timers);
            delete(timers);
        end
        delete(figHandle);
    end
end