function IKMouseCOMTool()
    links = struct('L1', 150, 'L2', 250, 'L3', 250, 'L4', 150, 'L5', 120);
    defaultPose = [60, 350];
    leftTarget = defaultPose;
    rightTarget = defaultPose;
    leftTrace = leftTarget;
    rightTrace = rightTarget;
    leftPath = [];
    rightPath = [];
    dragLeg = '';
    currentLeftIK = solveIK(leftTarget(1), leftTarget(2), links);
    currentRightIK = solveIK(rightTarget(1), rightTarget(2), links);

    portObj = [];
    useLegacySerial = false;
    txTimer = [];
    pathTimer = [];
    pathIndex = 1;

    fig = figure( ...
        'Name', 'Wheel Leg IK Mouse COM Tool', ...
        'NumberTitle', 'off', ...
        'Color', [0.96 0.97 0.99], ...
        'Position', [60 50 1620 900], ...
        'MenuBar', 'none', ...
        'ToolBar', 'figure', ...
        'WindowButtonDownFcn', @onMouseDown, ...
        'WindowButtonMotionFcn', @onMouseMove, ...
        'WindowButtonUpFcn', @onMouseUp, ...
        'CloseRequestFcn', @onCloseFigure);

    leftView = createLegView(fig, [0.29 0.54 0.31 0.4], 'Left Leg', [0.13 0.58 0.95], [0.83 0.28 0.23]);
    rightView = createLegView(fig, [0.63 0.54 0.31 0.4], 'Right Leg', [0.27 0.67 0.35], [0.89 0.50 0.18]);

    controlX = 0.02;
    labelW = 0.07;
    editW = 0.09;
    buttonW = 0.09;
    rowH = 0.035;
    row = 0.94;
    gap = 0.042;

    makeSectionTitle('Links', row);
    row = row - gap;
    [~, editL1] = addLabelEdit('L1', links.L1, row, @onLinkEdit);
    row = row - gap;
    [~, editL2] = addLabelEdit('L2', links.L2, row, @onLinkEdit);
    row = row - gap;
    [~, editL3] = addLabelEdit('L3', links.L3, row, @onLinkEdit);
    row = row - gap;
    [~, editL4] = addLabelEdit('L4', links.L4, row, @onLinkEdit);
    row = row - gap;
    [~, editL5] = addLabelEdit('L5', links.L5, row, @onLinkEdit);

    row = row - 0.055;
    makeSectionTitle('Targets', row);
    row = row - gap;
    [~, editLeftX] = addLabelEdit('LX', leftTarget(1), row, @onTargetEdit);
    row = row - gap;
    [~, editLeftY] = addLabelEdit('LY', leftTarget(2), row, @onTargetEdit);
    row = row - gap;
    [~, editRightX] = addLabelEdit('RX', rightTarget(1), row, @onTargetEdit);
    row = row - gap;
    [~, editRightY] = addLabelEdit('RY', rightTarget(2), row, @onTargetEdit);

    row = row - gap;
    [~, editDefaultX] = addLabelEdit('DX', defaultPose(1), row, @onDefaultPoseEdit);
    row = row - gap;
    [~, editDefaultY] = addLabelEdit('DY', defaultPose(2), row, @onDefaultPoseEdit);

    row = row - 0.055;
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [controlX row buttonW rowH], 'String', 'Default pose', 'Callback', @onDefaultPose);
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [controlX + buttonW + 0.01 row buttonW rowH], 'String', 'Clear trace', 'Callback', @onClearTrace);

    row = row - 0.07;
    makeSectionTitle('COM / TX', row);
    row = row - gap;
    [~, editPort] = addLabelEdit('Port', 'COM5', row, []);
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [controlX + labelW + editW + 0.01 row 0.08 rowH], 'String', 'Scan', 'Callback', @onScanPorts);

    row = row - gap;
    uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [controlX row labelW 0.028], 'String', 'List', ...
        'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left');
    popupPorts = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
        'Position', [controlX + labelW row 0.18 0.036], 'String', {'<none>'}, ...
        'BackgroundColor', [1 1 1], 'Callback', @onPortSelected);

    row = row - gap;
    [~, editBaud] = addLabelEdit('Baud', 115200, row, []);
    row = row - gap;
    [~, editTxHz] = addLabelEdit('TX Hz', 20, row, @onTxRateEdit);

    row = row - gap;
    checkAutoLeft = uicontrol(fig, 'Style', 'checkbox', 'Units', 'normalized', ...
        'Position', [controlX row 0.12 0.03], 'Value', 1, 'String', 'Auto left', ...
        'BackgroundColor', fig.Color, 'Callback', @onAutoSendChanged);
    checkAutoRight = uicontrol(fig, 'Style', 'checkbox', 'Units', 'normalized', ...
        'Position', [controlX + 0.12 row 0.12 0.03], 'Value', 1, 'String', 'Auto right', ...
        'BackgroundColor', fig.Color, 'Callback', @onAutoSendChanged);

    row = row - gap;
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [controlX row buttonW rowH], 'String', 'Connect', 'Callback', @onConnect);
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [controlX + buttonW + 0.01 row buttonW rowH], 'String', 'Disconnect', 'Callback', @onDisconnect);

    row = row - gap;
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [controlX row 0.075 rowH], 'String', 'Send L', 'Callback', @(~,~) trySend(false, 'left'));
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [controlX + 0.085 row 0.075 rowH], 'String', 'Send R', 'Callback', @(~,~) trySend(false, 'right'));
    uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [controlX + 0.17 row 0.075 rowH], 'String', 'Send both', 'Callback', @(~,~) trySend(false, 'both'));

    row = row - 0.06;
    txtPortStatus = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [controlX row 0.27 0.05], 'String', 'COM: disconnected', ...
        'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left', 'ForegroundColor', [0.18 0.20 0.24]);

    row = row - 0.07;
    txtCmdPreview = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [controlX row 0.25 0.075], 'String', sprintf('Left : -\nRight: -'), ...
        'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left', ...
        'FontName', 'Consolas', 'ForegroundColor', [0.18 0.20 0.24]);

    pathPanel = uipanel(fig, 'Title', 'Path Planner', 'Units', 'normalized', ...
        'Position', [0.29 0.05 0.65 0.32], 'BackgroundColor', fig.Color, ...
        'FontWeight', 'bold', 'FontSize', 11);

    [~, popupPathLeg] = addPanelPopup(pathPanel, 'Leg', {'Left', 'Right', 'Both'}, [0.03 0.76 0.18 0.14], @onPathConfigChanged);
    [~, popupShape] = addPanelPopup(pathPanel, 'Shape', {'Circle', 'Square', 'Rectangle'}, [0.26 0.76 0.18 0.14], @onPathConfigChanged);
    [~, editPathCenterX] = addPanelEdit(pathPanel, 'CX', 60, [0.49 0.76 0.12 0.14], @onPathConfigChanged);
    [~, editPathCenterY] = addPanelEdit(pathPanel, 'CY', 220, [0.64 0.76 0.12 0.14], @onPathConfigChanged);
    [~, editPathRadius] = addPanelEdit(pathPanel, 'R', 40, [0.79 0.76 0.12 0.14], @onPathConfigChanged);

    [~, editPathWidth] = addPanelEdit(pathPanel, 'W', 80, [0.03 0.50 0.12 0.14], @onPathConfigChanged);
    [~, editPathHeight] = addPanelEdit(pathPanel, 'H', 120, [0.18 0.50 0.12 0.14], @onPathConfigChanged);
    [~, editPathPoints] = addPanelEdit(pathPanel, 'Pts', 120, [0.33 0.50 0.12 0.14], @onPathConfigChanged);
    [~, editPathHz] = addPanelEdit(pathPanel, 'PlayHz', 30, [0.48 0.50 0.12 0.14], @onPathHzEdit);

    checkLoopPath = uicontrol(pathPanel, 'Style', 'checkbox', 'Units', 'normalized', ...
        'Position', [0.63 0.55 0.14 0.10], 'Value', 1, 'String', 'Loop path', ...
        'BackgroundColor', fig.Color);

    uicontrol(pathPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.78 0.54 0.08 0.12], 'String', 'Generate', 'Callback', @onGeneratePath);
    uicontrol(pathPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.87 0.54 0.06 0.12], 'String', 'Play', 'Callback', @onPlayPath);
    uicontrol(pathPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.94 0.54 0.05 0.12], 'String', 'Stop', 'Callback', @onStopPath);

    uicontrol(pathPanel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.03 0.24 0.30 0.14], 'String', 'Circle: use CX, CY, R', ...
        'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left', 'ForegroundColor', [0.25 0.27 0.32]);
    uicontrol(pathPanel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.33 0.24 0.33 0.14], 'String', 'Square: use CX, CY, W', ...
        'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left', 'ForegroundColor', [0.25 0.27 0.32]);
    uicontrol(pathPanel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.64 0.24 0.32 0.14], 'String', 'Rectangle: use CX, CY, W, H', ...
        'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left', 'ForegroundColor', [0.25 0.27 0.32]);

    txtPathStatus = uicontrol(pathPanel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.03 0.04 0.94 0.12], 'String', 'Path: idle', ...
        'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left', 'ForegroundColor', [0.18 0.20 0.24]);

    scanPorts();
    refreshAll();

    function makeSectionTitle(titleText, yPos)
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [controlX yPos 0.24 0.03], 'String', titleText, ...
            'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left', ...
            'FontWeight', 'bold', 'FontSize', 11);
    end

    function [labelHandle, editHandle] = addLabelEdit(labelText, defaultValue, yPos, callbackFn)
        labelHandle = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [controlX yPos labelW 0.028], 'String', labelText, ...
            'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left');

        if ischar(defaultValue) || isstring(defaultValue)
            valueText = char(defaultValue);
        else
            valueText = num2str(defaultValue);
        end

        editHandle = uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [controlX + labelW yPos editW 0.036], 'String', valueText, ...
            'BackgroundColor', [1 1 1]);

        if ~isempty(callbackFn)
            set(editHandle, 'Callback', callbackFn);
        end
    end

    function [labelHandle, editHandle] = addPanelEdit(parentHandle, labelText, defaultValue, boxPos, callbackFn)
        labelHandle = uicontrol(parentHandle, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [boxPos(1) boxPos(2) 0.05 0.10], 'String', labelText, ...
            'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left');

        editHandle = uicontrol(parentHandle, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [boxPos(1) + 0.05 boxPos(2) boxPos(3) 0.12], 'String', num2str(defaultValue), ...
            'BackgroundColor', [1 1 1]);

        if ~isempty(callbackFn)
            set(editHandle, 'Callback', callbackFn);
        end
    end

    function [labelHandle, popupHandle] = addPanelPopup(parentHandle, labelText, items, boxPos, callbackFn)
        labelHandle = uicontrol(parentHandle, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [boxPos(1) boxPos(2) 0.07 0.10], 'String', labelText, ...
            'BackgroundColor', fig.Color, 'HorizontalAlignment', 'left');

        popupHandle = uicontrol(parentHandle, 'Style', 'popupmenu', 'Units', 'normalized', ...
            'Position', [boxPos(1) + 0.07 boxPos(2) boxPos(3) 0.12], 'String', items, ...
            'BackgroundColor', [1 1 1]);

        if ~isempty(callbackFn)
            set(popupHandle, 'Callback', callbackFn);
        end
    end

    function onLinkEdit(~, ~)
        links.L1 = readPositive(editL1, links.L1);
        links.L2 = readPositive(editL2, links.L2);
        links.L3 = readPositive(editL3, links.L3);
        links.L4 = readPositive(editL4, links.L4);
        links.L5 = readPositive(editL5, links.L5);
        refreshAll();
    end

    function onTargetEdit(~, ~)
        leftTarget = [readNumber(editLeftX, leftTarget(1)), readNumber(editLeftY, leftTarget(2))];
        rightTarget = [readNumber(editRightX, rightTarget(1)), readNumber(editRightY, rightTarget(2))];
        appendTrace('left', leftTarget);
        appendTrace('right', rightTarget);
        refreshAll();
    end

    function onDefaultPoseEdit(~, ~)
        defaultPose = [readNumber(editDefaultX, defaultPose(1)), readNumber(editDefaultY, defaultPose(2))];
        set(editDefaultX, 'String', sprintf('%.2f', defaultPose(1)));
        set(editDefaultY, 'String', sprintf('%.2f', defaultPose(2)));
    end

    function onDefaultPose(~, ~)
        stopPathPlayback();
        defaultPose = [readNumber(editDefaultX, defaultPose(1)), readNumber(editDefaultY, defaultPose(2))];
        leftTarget = defaultPose;
        rightTarget = defaultPose;
        appendTrace('left', leftTarget);
        appendTrace('right', rightTarget);
        refreshAll();
    end

    function onClearTrace(~, ~)
        leftTrace = leftTarget;
        rightTrace = rightTarget;
        refreshAll();
    end

    function onMouseDown(~, ~)
        clickedObject = hittest(fig);
        clickedAxes = ancestor(clickedObject, 'axes');
        if isempty(clickedAxes)
            return;
        end

        if clickedAxes == leftView.ax
            dragLeg = 'left';
            stopPathPlayback();
            setLegTarget('left', readMousePoint(leftView.ax), true);
        elseif clickedAxes == rightView.ax
            dragLeg = 'right';
            stopPathPlayback();
            setLegTarget('right', readMousePoint(rightView.ax), true);
        else
            dragLeg = '';
        end
    end

    function onMouseMove(~, ~)
        switch dragLeg
            case 'left'
                setLegTarget('left', readMousePoint(leftView.ax), true);
            case 'right'
                setLegTarget('right', readMousePoint(rightView.ax), true);
        end
    end

    function onMouseUp(~, ~)
        dragLeg = '';
    end

    function point = readMousePoint(axHandle)
        currentPoint = get(axHandle, 'CurrentPoint');
        point = currentPoint(1, 1:2);
        xBounds = xlim(axHandle);
        yBounds = ylim(axHandle);
        point(1) = min(max(point(1), xBounds(1)), xBounds(2));
        point(2) = min(max(point(2), yBounds(1)), yBounds(2));
    end

    function setLegTarget(legName, point, addToTrace)
        switch legName
            case 'left'
                leftTarget = point;
                if addToTrace
                    appendTrace('left', leftTarget);
                end
            case 'right'
                rightTarget = point;
                if addToTrace
                    appendTrace('right', rightTarget);
                end
        end
        refreshAll();
    end

    function appendTrace(legName, point)
        switch legName
            case 'left'
                if isempty(leftTrace) || norm(leftTrace(end, :) - point) > 0.5
                    leftTrace = [leftTrace; point]; %#ok<AGROW>
                end
            case 'right'
                if isempty(rightTrace) || norm(rightTrace(end, :) - point) > 0.5
                    rightTrace = [rightTrace; point]; %#ok<AGROW>
                end
        end
    end

    function refreshAll()
        currentLeftIK = solveIK(leftTarget(1), leftTarget(2), links);
        currentRightIK = solveIK(rightTarget(1), rightTarget(2), links);

        set(editL1, 'String', num2str(links.L1));
        set(editL2, 'String', num2str(links.L2));
        set(editL3, 'String', num2str(links.L3));
        set(editL4, 'String', num2str(links.L4));
        set(editL5, 'String', num2str(links.L5));
        set(editLeftX, 'String', sprintf('%.2f', leftTarget(1)));
        set(editLeftY, 'String', sprintf('%.2f', leftTarget(2)));
        set(editRightX, 'String', sprintf('%.2f', rightTarget(1)));
        set(editRightY, 'String', sprintf('%.2f', rightTarget(2)));
        set(editDefaultX, 'String', sprintf('%.2f', defaultPose(1)));
        set(editDefaultY, 'String', sprintf('%.2f', defaultPose(2)));

        updateLegView(leftView, leftTarget, leftTrace, leftPath, currentLeftIK, 'Left');
        updateLegView(rightView, rightTarget, rightTrace, rightPath, currentRightIK, 'Right');

        leftCommand = buildCommand('left');
        rightCommand = buildCommand('right');
        set(txtCmdPreview, 'String', sprintf('Left : %s\nRight: %s', strtrim(leftCommand), strtrim(rightCommand)));
        set(txtPortStatus, 'String', portStatusString());
        drawnow limitrate;
    end

    function updateLegView(view, point, traceData, pathData, ikResult, legLabel)
        set(view.pointB, 'XData', point(1), 'YData', point(2));
        set(view.trace, 'XData', traceData(:, 1), 'YData', traceData(:, 2));
        set(view.baseD, 'XData', links.L5, 'YData', 0);

        if isempty(pathData)
            set(view.path, 'XData', NaN, 'YData', NaN);
        else
            set(view.path, 'XData', pathData(:, 1), 'YData', pathData(:, 2));
        end

        if ikResult.valid
            set(view.l1, 'XData', [0 ikResult.A(1)], 'YData', [0 ikResult.A(2)]);
            set(view.l2, 'XData', [ikResult.A(1) ikResult.B(1)], 'YData', [ikResult.A(2) ikResult.B(2)]);
            set(view.l3, 'XData', [ikResult.C(1) ikResult.B(1)], 'YData', [ikResult.C(2) ikResult.B(2)]);
            set(view.l4, 'XData', [ikResult.D(1) ikResult.C(1)], 'YData', [ikResult.D(2) ikResult.C(2)]);
            set(view.l5, 'XData', [0 ikResult.D(1)], 'YData', [0 ikResult.D(2)]);
            set(view.pointA, 'XData', ikResult.A(1), 'YData', ikResult.A(2));
            set(view.pointC, 'XData', ikResult.C(1), 'YData', ikResult.C(2));
            set(view.pointB, 'MarkerFaceColor', view.validColor, 'MarkerEdgeColor', [0.08 0.12 0.18]);
            set(view.info, 'String', sprintf(['%s Target: (%.2f, %.2f)\n' ...
                                              'Alpha      %.2f deg\n' ...
                                              'Beta       %.2f deg'], ...
                                              legLabel, point(1), point(2), ...
                                              rad2deg(ikResult.alpha), rad2deg(ikResult.beta)));
            set(view.status, 'String', 'IK valid');
            set(view.status, 'Color', [0.12 0.45 0.20]);
        else
            clearLegView(view);
            set(view.pointB, 'MarkerFaceColor', [0.89 0.30 0.26], 'MarkerEdgeColor', [0.45 0.08 0.08]);
            set(view.info, 'String', sprintf(['%s Target: (%.2f, %.2f)\n' ...
                                              'Alpha      --\n' ...
                                              'Beta       --'], ...
                                              legLabel, point(1), point(2)));
            set(view.status, 'String', 'IK invalid');
            set(view.status, 'Color', [0.72 0.18 0.14]);
        end
    end

    function clearLegView(view)
        set(view.l1, 'XData', [0 NaN], 'YData', [0 NaN]);
        set(view.l2, 'XData', [NaN NaN], 'YData', [NaN NaN]);
        set(view.l3, 'XData', [NaN NaN], 'YData', [NaN NaN]);
        set(view.l4, 'XData', [links.L5 NaN], 'YData', [0 NaN]);
        set(view.l5, 'XData', [0 links.L5], 'YData', [0 0]);
        set(view.pointA, 'XData', NaN, 'YData', NaN);
        set(view.pointC, 'XData', NaN, 'YData', NaN);
    end

    function onScanPorts(~, ~)
        scanPorts();
    end

    function scanPorts()
        ports = listAvailablePorts();
        if isempty(ports)
            ports = {'<none>'};
        end
        ports = normalizePortList(ports);
        set(popupPorts, 'String', ports, 'Value', 1);
        if ~strcmp(ports{1}, '<none>')
            set(editPort, 'String', ports{1});
            set(txtPortStatus, 'String', sprintf('COM: found %d port(s): %s', numel(ports), strjoin(ports, ', ')));
        else
            set(txtPortStatus, 'String', 'COM: no ports found');
        end
    end

    function onPortSelected(~, ~)
        ports = cellstr(get(popupPorts, 'String'));
        selected = ports{get(popupPorts, 'Value')};
        if ~strcmp(selected, '<none>')
            set(editPort, 'String', selected);
        end
    end

    function onConnect(~, ~)
        portName = strtrim(get(editPort, 'String'));
        baudRate = round(readPositive(editBaud, 115200));

        if isempty(portName)
            set(txtPortStatus, 'String', 'COM: port is empty');
            return;
        end

        closePort();

        try
            if exist('serialport', 'file') == 2
                portObj = serialport(portName, baudRate, 'Timeout', 0.05);
                configureTerminator(portObj, "LF");
                flush(portObj);
                useLegacySerial = false;
            else
                portObj = serial(portName, 'BaudRate', baudRate, 'Terminator', 'LF'); %#ok<SERIAL>
                fopen(portObj);
                useLegacySerial = true;
            end
            set(txtPortStatus, 'String', sprintf('COM: connected %s @ %d', portName, baudRate));
        catch ME
            portObj = [];
            useLegacySerial = false;
            set(txtPortStatus, 'String', sprintf('COM: connect failed (%s)', ME.message));
        end

        updateTxTimer();
        refreshAll();
    end

    function onDisconnect(~, ~)
        closePort();
        refreshAll();
    end

    function closePort()
        stopTxTimer();

        if isempty(portObj)
            return;
        end

        try
            if useLegacySerial
                if strcmp(portObj.Status, 'open')
                    fclose(portObj);
                end
                delete(portObj);
            else
                delete(portObj);
            end
        catch
        end

        portObj = [];
        useLegacySerial = false;
    end

    function onAutoSendChanged(~, ~)
        updateTxTimer();
    end

    function onTxRateEdit(~, ~)
        updateTxTimer();
    end

    function updateTxTimer()
        stopTxTimer();

        if isempty(portObj)
            return;
        end

        if get(checkAutoLeft, 'Value') == 0 && get(checkAutoRight, 'Value') == 0
            return;
        end

        txRate = max(1, round(readPositive(editTxHz, 20)));
        txTimer = timer( ...
            'ExecutionMode', 'fixedSpacing', ...
            'Period', 1 / txRate, ...
            'BusyMode', 'drop', ...
            'TimerFcn', @onTxTimerTick);
        start(txTimer);
    end

    function stopTxTimer()
        if isempty(txTimer)
            return;
        end
        try
            stop(txTimer);
            delete(txTimer);
        catch
        end
        txTimer = [];
    end

    function onTxTimerTick(~, ~)
        if ~ishandle(fig)
            return;
        end
        trySend(true, 'auto');
    end

    function trySend(fromTimer, sendMode)
        if isempty(portObj)
            if ~fromTimer
                set(txtPortStatus, 'String', 'COM: not connected');
            end
            return;
        end

        switch sendMode
            case 'auto'
                sendLeft = get(checkAutoLeft, 'Value') == 1;
                sendRight = get(checkAutoRight, 'Value') == 1;
            case 'left'
                sendLeft = true;
                sendRight = false;
            case 'right'
                sendLeft = false;
                sendRight = true;
            otherwise
                sendLeft = true;
                sendRight = true;
        end

        sentLabels = {};
        if sendLeft && currentLeftIK.valid
            sendCommand(buildCommand('left'));
            sentLabels{end + 1} = 'L'; %#ok<AGROW>
        end
        if sendRight && currentRightIK.valid
            sendCommand(buildCommand('right'));
            sentLabels{end + 1} = 'R'; %#ok<AGROW>
        end

        if ~fromTimer
            if isempty(sentLabels)
                set(txtPortStatus, 'String', 'COM: nothing sent (IK invalid or disabled)');
            else
                set(txtPortStatus, 'String', sprintf('COM: sent %s', strjoin(sentLabels, '+')));
            end
        end
    end

    function sendCommand(commandText)
        if useLegacySerial
            fprintf(portObj, '%s', commandText);
        else
            writeline(portObj, strtrim(commandText));
        end
    end

    function commandText = buildCommand(legName)
        switch legName
            case 'left'
                commandText = sprintf('ikleft,%.2f,%.2f\n', leftTarget(1), leftTarget(2));
            case 'right'
                commandText = sprintf('ikright,%.2f,%.2f\n', rightTarget(1), rightTarget(2));
            otherwise
                commandText = sprintf('ik,%.2f,%.2f,%.2f,%.2f\n', leftTarget(1), leftTarget(2), rightTarget(1), rightTarget(2));
        end
    end

    function textOut = portStatusString()
        if isempty(portObj)
            textOut = 'COM: disconnected';
        else
            textOut = sprintf('COM: connected (%s)', strtrim(get(editPort, 'String')));
        end
    end

    function onPathConfigChanged(~, ~)
        updateGeneratedPath(false);
    end

    function onPathHzEdit(~, ~)
        if ~isempty(pathTimer)
            restartPathPlayback();
        end
    end

    function onGeneratePath(~, ~)
        updateGeneratedPath(true);
    end

    function updateGeneratedPath(updateStatus)
        pathShapeList = cellstr(get(popupShape, 'String'));
        selectedShape = pathShapeList{get(popupShape, 'Value')};
        cx = readNumber(editPathCenterX, 60);
        cy = readNumber(editPathCenterY, 220);
        radius = readPositive(editPathRadius, 40);
        width = readPositive(editPathWidth, 80);
        height = readPositive(editPathHeight, 120);
        pointCount = max(16, round(readPositive(editPathPoints, 120)));
        generated = generatePathPoints(selectedShape, cx, cy, radius, width, height, pointCount);

        legList = cellstr(get(popupPathLeg, 'String'));
        selectedLeg = legList{get(popupPathLeg, 'Value')};
        switch selectedLeg
            case 'Left'
                leftPath = generated;
            case 'Right'
                rightPath = generated;
            otherwise
                leftPath = generated;
                rightPath = generated;
        end

        if updateStatus
            set(txtPathStatus, 'String', sprintf('Path: %s generated for %s', selectedShape, selectedLeg));
        end
        refreshAll();
    end

    function onPlayPath(~, ~)
        if isempty(leftPath) && isempty(rightPath)
            updateGeneratedPath(true);
        end

        if isempty(leftPath) && isempty(rightPath)
            set(txtPathStatus, 'String', 'Path: no path to play');
            return;
        end

        restartPathPlayback();
    end

    function restartPathPlayback()
        stopPathPlayback();
        pathIndex = 1;
        playHz = max(1, round(readPositive(editPathHz, 30)));
        pathTimer = timer( ...
            'ExecutionMode', 'fixedSpacing', ...
            'Period', 1 / playHz, ...
            'BusyMode', 'drop', ...
            'TimerFcn', @onPathTimerTick);
        start(pathTimer);
        set(txtPathStatus, 'String', 'Path: playing');
    end

    function onPathTimerTick(~, ~)
        if ~ishandle(fig)
            return;
        end

        maxLength = max(size(leftPath, 1), size(rightPath, 1));
        if maxLength == 0
            stopPathPlayback();
            return;
        end

        if ~isempty(leftPath)
            leftTarget = leftPath(mod(pathIndex - 1, size(leftPath, 1)) + 1, :);
            appendTrace('left', leftTarget);
        end
        if ~isempty(rightPath)
            rightTarget = rightPath(mod(pathIndex - 1, size(rightPath, 1)) + 1, :);
            appendTrace('right', rightTarget);
        end

        refreshAll();
        pathIndex = pathIndex + 1;

        if pathIndex > maxLength
            if get(checkLoopPath, 'Value') == 1
                pathIndex = 1;
            else
                stopPathPlayback();
                set(txtPathStatus, 'String', 'Path: completed');
            end
        end
    end

    function onStopPath(~, ~)
        stopPathPlayback();
        set(txtPathStatus, 'String', 'Path: stopped');
    end

    function stopPathPlayback()
        if isempty(pathTimer)
            return;
        end
        try
            stop(pathTimer);
            delete(pathTimer);
        catch
        end
        pathTimer = [];
    end

    function onCloseFigure(~, ~)
        stopPathPlayback();
        closePort();
        if ishandle(fig)
            delete(fig);
        end
    end
end

function view = createLegView(parentFig, position, titleText, pointColor, pathColor)
    view.ax = axes('Parent', parentFig, 'Position', position);
    hold(view.ax, 'on');
    grid(view.ax, 'on');
    axis(view.ax, 'equal');
    xlim(view.ax, [-180 320]);
    ylim(view.ax, [-20 420]);
    set(view.ax, 'YDir', 'reverse');
    title(view.ax, titleText);
    xlabel(view.ax, 'X');
    ylabel(view.ax, 'Y');

    linkColor = [0.10 0.12 0.16];
    view.l1 = plot(view.ax, [0 0], [0 0], '-', 'Color', linkColor, 'LineWidth', 3);
    view.l2 = plot(view.ax, [0 0], [0 0], '-', 'Color', linkColor, 'LineWidth', 3);
    view.l3 = plot(view.ax, [0 0], [0 0], '-', 'Color', linkColor, 'LineWidth', 3);
    view.l4 = plot(view.ax, [0 0], [0 0], '-', 'Color', linkColor, 'LineWidth', 3);
    view.l5 = plot(view.ax, [0 0], [0 0], '-', 'Color', [0.55 0.58 0.64], 'LineWidth', 5);
    view.trace = plot(view.ax, NaN, NaN, '-', 'Color', pathColor, 'LineWidth', 1.2);
    view.path = plot(view.ax, NaN, NaN, '--', 'Color', [0.40 0.40 0.46], 'LineWidth', 1.5);
    view.pointB = plot(view.ax, NaN, NaN, 'o', 'MarkerFaceColor', pointColor, 'MarkerEdgeColor', [0.08 0.12 0.18], 'MarkerSize', 10);
    view.pointA = plot(view.ax, NaN, NaN, 'o', 'MarkerFaceColor', [0.93 0.66 0.18], 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
    view.pointC = plot(view.ax, NaN, NaN, 'o', 'MarkerFaceColor', [0.18 0.72 0.42], 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
    view.baseO = plot(view.ax, 0, 0, 'o', 'MarkerFaceColor', [0.12 0.12 0.12], 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
    view.baseD = plot(view.ax, NaN, NaN, 'o', 'MarkerFaceColor', [0.12 0.12 0.12], 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
    view.info = text(view.ax, -170, 22, '', 'FontSize', 11, 'VerticalAlignment', 'top', 'FontName', 'Consolas');
    view.status = text(view.ax, -170, 88, '', 'FontSize', 10, 'VerticalAlignment', 'top', 'FontName', 'Consolas');
    view.validColor = pointColor;
end

function result = solveIK(X, Y, links)
    a = 2 * X * links.L1;
    b = 2 * Y * links.L1;
    c = X^2 + Y^2 + links.L1^2 - links.L2^2;

    d = 2 * links.L4 * (X - links.L5);
    e = 2 * links.L4 * Y;
    f = (X - links.L5)^2 + links.L4^2 + Y^2 - links.L3^2;

    alphaDisc = a^2 + b^2 - c^2;
    betaDisc = d^2 + e^2 - f^2;

    result.valid = isfinite(alphaDisc) && isfinite(betaDisc) && alphaDisc >= 0 && betaDisc >= 0;
    result.alpha = NaN;
    result.beta = NaN;
    result.A = [NaN NaN];
    result.B = [X Y];
    result.C = [NaN NaN];
    result.D = [links.L5 0];

    if ~result.valid
        return;
    end

    alpha1 = mod(2 * atan2(b + sqrt(alphaDisc), a + c), 2 * pi);
    alpha2 = mod(2 * atan2(b - sqrt(alphaDisc), a + c), 2 * pi);
    beta1 = mod(2 * atan2(e + sqrt(betaDisc), d + f), 2 * pi);
    beta2 = mod(2 * atan2(e - sqrt(betaDisc), d + f), 2 * pi);

    if alpha1 >= pi / 4
        alpha = alpha1;
    else
        alpha = alpha2;
    end

    if beta1 >= 0 && beta1 <= pi / 4
        beta = beta1;
    else
        beta = beta2;
    end

    result.alpha = alpha;
    result.beta = beta;
    result.alpha1 = alpha1;
    result.alpha2 = alpha2;
    result.beta1 = beta1;
    result.beta2 = beta2;
    result.A = [links.L1 * cos(alpha), links.L1 * sin(alpha)];
    result.C = [links.L5 + links.L4 * cos(beta), links.L4 * sin(beta)];
end

function value = readNumber(editHandle, fallback)
    value = str2double(get(editHandle, 'String'));
    if ~isfinite(value)
        value = fallback;
    end
end

function value = readPositive(editHandle, fallback)
    value = str2double(get(editHandle, 'String'));
    if ~isfinite(value) || value <= 0
        value = fallback;
    end
end

function ports = listAvailablePorts()
    ports = {};

    try
        if exist('serialportlist', 'file') == 2 || exist('serialportlist', 'builtin') == 5
            rawPorts = serialportlist("available");
            ports = normalizePortList(rawPorts);
        end
    catch
    end

    try
        if isempty(ports) && (exist('serialportlist', 'file') == 2 || exist('serialportlist', 'builtin') == 5)
            rawPorts = serialportlist("all");
            ports = normalizePortList(rawPorts);
        end
    catch
    end

    try
        hw = instrhwinfo('serial');
        if isfield(hw, 'AvailableSerialPorts')
            ports = [ports, normalizePortList(hw.AvailableSerialPorts)]; %#ok<AGROW>
        end
    catch
    end

    try
        [status, cmdout] = system('mode');
        if status == 0
            modePorts = regexp(cmdout, 'COM\d+', 'match');
            ports = [ports, normalizePortList(modePorts)]; %#ok<AGROW>
        end
    catch
    end

    try
        [status, cmdout] = system('powershell -NoProfile -Command "[System.IO.Ports.SerialPort]::GetPortNames()"');
        if status == 0
            dotnetPorts = regexp(cmdout, 'COM\d+', 'match');
            ports = [ports, normalizePortList(dotnetPorts)]; %#ok<AGROW>
        end
    catch
    end

    ports = normalizePortList(ports);
end

function ports = normalizePortList(rawPorts)
    if isempty(rawPorts)
        ports = {};
        return;
    end

    if isstring(rawPorts)
        ports = cellstr(rawPorts);
    elseif ischar(rawPorts)
        ports = cellstr(rawPorts);
    else
        ports = rawPorts;
    end

    ports = ports(:)';
    ports = ports(~cellfun(@isempty, ports));
    ports = cellfun(@strtrim, ports, 'UniformOutput', false);
    ports = ports(~cellfun(@isempty, ports));
    ports = unique(ports, 'stable');
end

function pathPoints = generatePathPoints(shapeName, cx, cy, radius, width, height, pointCount)
    theta = linspace(0, 2 * pi, pointCount + 1)';
    theta(end) = [];

    switch lower(shapeName)
        case 'circle'
            pathPoints = [cx + radius * cos(theta), cy + radius * sin(theta)];

        case 'square'
            pathPoints = rectanglePath(cx, cy, width, width, pointCount);

        otherwise
            pathPoints = rectanglePath(cx, cy, width, height, pointCount);
    end
end

function points = rectanglePath(cx, cy, width, height, pointCount)
    corners = [ ...
        cx - width / 2, cy - height / 2;
        cx + width / 2, cy - height / 2;
        cx + width / 2, cy + height / 2;
        cx - width / 2, cy + height / 2;
        cx - width / 2, cy - height / 2];

    edgeCount = size(corners, 1) - 1;
    baseCount = floor(pointCount / edgeCount);
    remainder = pointCount - baseCount * edgeCount;
    points = zeros(pointCount, 2);
    idx = 1;

    for edgeIdx = 1:edgeCount
        samples = baseCount + (edgeIdx <= remainder);
        t = linspace(0, 1, samples + 1)';
        if edgeIdx < edgeCount
            t(end) = [];
        end
        segment = corners(edgeIdx, :) + (corners(edgeIdx + 1, :) - corners(edgeIdx, :)) .* t;
        count = size(segment, 1);
        points(idx:idx + count - 1, :) = segment;
        idx = idx + count;
    end

    points = points(1:pointCount, :);
end
