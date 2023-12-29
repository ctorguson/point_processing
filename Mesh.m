function Mesh()

% Add the build directory to the MATLAB path
currentDir = pwd;
% addpath(fullfile(currentDir, 'build'));
addpath('/Users/ciaratorguson/point_processing/build');
pyenv('Version', '/Users/ciaratorguson/myenv2/bin/python', 'ExecutionMode', 'InProcess');

% Create figure
f = figure('Name', 'Point Cloud Window', 'NumberTitle', 'off');
ax = axes(f);
set(ax, 'ButtonDownFcn', @(src, event) axes_ButtonDownFcn(src, event, f));
    function axes_ButtonDownFcn(~, event, f)
    % Check if pcData exists in f.UserData
    if ~isfield(f.UserData, 'pcData') || isempty(f.UserData.pcData)
        % Display a warning message if pcData doesn't exist
        warndlg('No point cloud data loaded!', 'Warning');
        return;
    end

    % Get the current point clicked
    currentPoint = event.IntersectionPoint(1:3);

    % Assuming f.UserData.pcData contains your point cloud
    % Find the closest point in your point cloud
    Mdl = KDTreeSearcher(f.UserData.pcData.Location);
    closestIndex = knnsearch(Mdl, currentPoint, 'K', 1);

    % Store the index of the closest point in UserData
    f.UserData.selectedPointIndex = closestIndex;
end

% Dropdown Menu for Opening Clouds
openMenu = uimenu(f, 'Label', 'Open');
newFormats = {'3D File', 'Merge Multiple'};
for i = 1:length(newFormats)
    uimenu(openMenu, 'Label', newFormats{i}, 'Callback', @(src, event) open_cloud_callback(src, event, f, ax, newFormats{i}));
end
% Dropdown Menu for Saving Options
saveMenu = uimenu(f, 'Label', 'Save Cloud As');
f.UserData.saveAllMenu = uimenu(saveMenu, 'Label', 'Save All Points', 'Enable', 'on', 'Callback', @(src, event) save_ply_callback(src, event, f, ax, 'all'));
f.UserData.saveBrushedMenu = uimenu(saveMenu, 'Label', 'Save Brushed Points', 'Enable', 'on', 'Callback', @(src, event) save_ply_callback(src, event, f, ax, 'brushed'));
% Dropdown Menu for Additional Visualization Options
vizOptionsMenu = uimenu(f, 'Label', 'Visualization');
depthOption = uimenu(vizOptionsMenu, 'Label', 'Depth', 'Callback', @(src, event) depth_option_callback(src, event, f, ax));
resetOption = uimenu(vizOptionsMenu, 'Label', 'Reset', 'Callback', @(src, event) reset_option_callback(src, event, f, ax));
meshOption = uimenu(vizOptionsMenu, 'Label', 'Mesh', 'Callback', @(src, event) mesh_option_callback(src, event, f, ax));
end

% Logic for "Reset" option
function reset_option_callback(~, ~, f, ax)
if isfield(f.UserData, 'originalPcData') && ~isempty(f.UserData.originalPcData)
    % Display original point cloud data
    pcshow(f.UserData.originalPcData, 'Parent', ax);
    f.UserData.pcData = f.UserData.originalPcData;
    title(ax, '');
    axis(ax, 'equal');
else
    errordlg('No original data to reset to!', 'Error');
end
end

function [vertices, faces] = read_off_file(filename)
    % Open the OFF file
    fileID = fopen(filename, 'r');
    
    % Read the header
    header = fgetl(fileID);
    if ~strcmp(header, 'OFF')
        error('File is not in the OFF format.');
    end
    
    % Read the number of vertices, faces, and edges
    info = fscanf(fileID, '%d %d %d', 3);
    numVertices = info(1);
    numFaces = info(2);
    
    % Read the vertex data
    vertices = fscanf(fileID, '%f %f %f', [3, numVertices])';
    
    % Initialize faces
    faces = cell(numFaces, 1);
    
    for i = 1:numFaces
        faceInfo = fscanf(fileID, '%d', 1);  % Number of vertices in the face
        face = fscanf(fileID, '%d', faceInfo);
        faces{i} = face' + 1;  % MATLAB is 1-based indexing
    end
    
    faces = cell2mat(faces);
    
    fclose(fileID);
end

function [vertices, faces] = point_processing(pc, f)
    % Convert point cloud data to a temporary .ply file
    tempFileNamePLY = 'tempPointCloud.ply';
    pcwrite(pc, tempFileNamePLY, 'Encoding', 'ASCII');

    % Call the PointProcessing MEX function using the .ply file
    disp(tempFileNamePLY);  % Print the file path for debugging
    try
        status = PointProcessing(tempFileNamePLY);
    catch ME
        disp('Error in PointProcessing:');
        disp(getReport(ME, 'extended'));  % Display extended error information
        rethrow(ME);
    end

    if status ~= 0
        error('Error in PointProcessing.');
    end
    
    % Check if the output_mesh.off file exists
    if ~exist('output_mesh.off', 'file')
        error('Mesh output file not found. Meshing may have failed.');
    end

    % Read the resulting mesh
    [vertices, faces] = read_off_file('output_mesh.off');

    % Clean up temporary files
    delete(tempFileNamePLY);
end

function visualize_mesh(vertices, faces, ax)
    patch('Faces', faces, 'Vertices', vertices, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'Parent', ax);
    view(3);
    axis equal;
    lighting gouraud;
    camlight;
end

function mesh_option_callback(~, ~, f, ax)
    % Ensure point cloud data exists
    if ~isfield(f.UserData, 'pcData') || isempty(f.UserData.pcData)
        errordlg('No point cloud data available.', 'Error');
        return;
    end

    % Process the point cloud data to get the mesh
    [meshVertices, meshFaces] = point_processing(f.UserData.pcData, f);

    % Visualize the mesh in the GUI
    visualize_mesh(meshVertices, meshFaces, ax);
end

function open_cloud_callback(~, ~, f, ax, option)
    switch option
        case '3D File'
            [file, path] = uigetfile({'*.pcd;*.ply;*.las;*.pts;*.xyz;*.pcd', 'Point Cloud Files'}, 'Open Cloud File');
            if isequal(file, 0)
                return;
            end
            fullPath = fullfile(path, file);
            
            % Store the selected file's path in UserData
            f.UserData.selectedFilePath = fullPath;

            % Determine the file extension to read the correct point cloud format
            [~, ~, ext] = fileparts(fullPath);
            pc = [];
            switch ext
                case '.ply'
                    pc = pcread(fullPath);
                otherwise
                    warning(['File format ', ext, ' is not supported.']);
                    return;
            end
            if ~isempty(pc)
                pcshow(pc, 'Parent', ax);
                f.UserData.pcData = pc;
                f.UserData.originalPcData = pc; % Store the original data
            end
    end
end

function save_ply_callback(~, ~, f, ax, saveMode)
pcData = f.UserData.pcData;
% Initialize file and path variables
file = [];
path = [];
if isempty(pcData)
    errordlg('No data to save!', 'Error');
    return;
end
% Determine the file extension based on saveMode
switch saveMode
    case 'all'
        [file, path] = uiputfile('*.ply', 'Save PLY File');
    case 'brushed'
        [file, path] = uiputfile('*.ply', 'Save PLY File');
    otherwise
        errordlg('Unknown save mode!', 'Error');
        if file == 0
            return; 'No Point'
        end
end
fullPath = fullfile(path, file);
if strcmp(saveMode, 'brushed')
    % Get brushed data
    axChildren = get(ax, 'Children');
    brushedIndices = [];
    for child = 1:length(axChildren)
        if isprop(axChildren(child), 'BrushData')
            currentBrushedIndices = find(axChildren(child).BrushData);
            brushedIndices = [brushedIndices; currentBrushedIndices];
        end
    end
    if isempty(brushedIndices)
        errordlg('No points were brushed!', 'Error');
        return;
    end
    % Display the number of brushed points
    disp(['Number of brushed points: ', num2str(length(brushedIndices))]);
    pcData = select(f.UserData.pcData, brushedIndices); % This also keeps RGB and intensity data intact
end

% Save the data based on file extension
[~, ~, option] = fileparts(fullPath);
switch option
    case '.ply'
        pcwrite(pcData, fullPath, 'Encoding', 'ascii');
    case '.asc'
        % Save in .asc format
        fileID = fopen(fullPath, 'w');
        for i = 1:size(pcData.Location, 1)
            fprintf(fileID, '%f %f %f\n', pcData.Location(i, 1), pcData.Location(i, 2), pcData.Location(i, 3));
        end
        fclose(fileID);
        pyenv('Version', 'system'); % Reset to system's default Python
    otherwise
        errordlg('Unsupported file type!', 'Error');
end
end