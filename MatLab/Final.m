% Import the URDF file and meshes
urdfPath = 'D:\WorkSpace\matlab_ws\v2_45_205\urdf\v2_45_205.urdf';
meshDir = 'D:\WorkSpace\matlab_ws\v2_45_205\meshes';
robot = importrobot(urdfPath, 'MeshPath', meshDir);
robot.DataFormat = 'struct';

% Create a figure for visualization
fig = figure('Name', 'Robot Joint Controller with STM32', 'Position', [100, 100, 900, 600]);
ax = axes('Parent', fig, 'Position', [0.35, 0.3, 0.6, 0.6]);
show(robot, 'Parent', ax, 'PreservePlot', false);
ax.CameraPositionMode = 'auto';

% Get the joint names dynamically
jointNames = cellfun(@(body) body.Joint.Name, robot.Bodies, 'UniformOutput', false);
jointNames = jointNames(~cellfun(@isempty, jointNames));

% Initialize configuration
q = robot.homeConfiguration;

% Store shared data in figure
setappdata(fig, 'robot', robot);
setappdata(fig, 'q', q);
setappdata(fig, 'jointNames', jointNames);

% Create sliders for each joint
sliderHandles = gobjects(numel(jointNames), 1);
sliderSpacing = linspace(450, 100, numel(jointNames));

for i = 1:numel(jointNames)
    sliderHandles(i) = uicontrol('Style', 'slider', ...
        'Min', -pi, 'Max', pi, ...
        'Value', q(i).JointPosition, ...
        'Position', [50, sliderSpacing(i), 200, 20], ...
        'Callback', @(src, event) updateJoint(fig, ax, i));
    
    uicontrol('Style', 'text', ...
        'Position', [10, sliderSpacing(i) + 5, 40, 15], ...
        'String', jointNames{i}, ...
        'HorizontalAlignment', 'right');
end
setappdata(fig, 'sliderHandles', sliderHandles);

% Define serial port parameters
port = "COM5";
baudRate = 115200;
serialObj = serialport(port, baudRate);
configureTerminator(serialObj, "LF");
flush(serialObj);

% Store serial object in figure
setappdata(fig, 'serialObj', serialObj);

% Configure serial callback for data received
configureCallback(serialObj, "terminator", @(src, event) serialCallback(src, fig, ax));

% Display start message
disp("Receiving data from STM32 and controlling joint...");

% Cleanup on figure close
set(fig, 'CloseRequestFcn', @(src, event) cleanup(fig));

% Nested updateJoint function (for sliders)
function updateJoint(fig, ax, jointIdx)
    robot = getappdata(fig, 'robot');
    q = getappdata(fig, 'q');
    sliderHandles = getappdata(fig, 'sliderHandles');
    jointNames = getappdata(fig, 'jointNames');

    if isvalid(sliderHandles(jointIdx))
        q(jointIdx).JointPosition = sliderHandles(jointIdx).Value;
        disp(['Joint ', jointNames{jointIdx}, ' set to: ', num2str(q(jointIdx).JointPosition)]);
    else
        warning(['Slider ', num2str(jointIdx), ' is not valid.']);
        return;
    end

    setappdata(fig, 'q', q);
    show(robot, q, 'Parent', ax, 'PreservePlot', false);
    drawnow;
end

% Serial data callback
function serialCallback(serialObj, fig, ax)
    % Read the incoming line
    data = readline(serialObj);
    disp("Received: " + data);

    % Check if this is the ECP Acc line
    if contains(data, "ECP Acc")
        % Extract X, Y, Z values (e.g., "ECP Acc  X: 1.20 Y: -0.50 Z: 0.80 (m/s²)")
        pattern = "X:\s*([-]?\d+\.\d+)\s*Y:\s*([-]?\d+\.\d+)\s*Z:\s*([-]?\d+\.\d+)";
        tokens = regexp(data, pattern, 'tokens');
        
        if ~isempty(tokens)
            zAccel = str2double(tokens{1}{3}); % Z value (3rd capture group)

            % Map Z acceleration (-9.8 to 9.8 m/s²) to joint range (-pi to pi)
            jointPos = mapRange(zAccel, -9.8, 9.8, -pi, pi);

            % Update joint 1 (change index if desired)
            q = getappdata(fig, 'q');
            q(1).JointPosition = jointPos; % Link Z to joint 1
            setappdata(fig, 'q', q);

            % Update slider position (optional)
            sliderHandles = getappdata(fig, 'sliderHandles');
            if isvalid(sliderHandles(1))
                sliderHandles(1).Value = jointPos;
            end

            % Update visualization
            robot = getappdata(fig, 'robot');
            show(robot, q, 'Parent', ax, 'PreservePlot', false);
            drawnow;

            disp(['Mapped Z Accel: ', num2str(zAccel), ' m/s² to Joint Pos: ', num2str(jointPos)]);
        else
            disp("Failed to parse ECP Acc line.");
        end
    end
end

% Helper function to map values
function out = mapRange(value, inMin, inMax, outMin, outMax)
    out = (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    out = max(outMin, min(outMax, out)); % Clamp to range
end

% Cleanup function
function cleanup(fig)
    serialObj = getappdata(fig, 'serialObj');
    if ~isempty(serialObj) && isvalid(serialObj)
        configureCallback(serialObj, "off"); % Disable callback
        clear serialObj; % Close serial port
    end
    delete(fig); % Close figure
end