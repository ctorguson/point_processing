function CustomMesh()

% dbstop if error;

% Add the build directory to the MATLAB path
% currentDir = pwd;mex -v -largeArrayDims /Users/ciaratorguson/point_processing/mainProcess.cpp -I/usr/local/Cellar/eigen/3.4.0_1/include/eigen3 -I/usr/local/opt/cgal/include -I/usr/local/opt/gmp/include -I/usr/local/opt/mpfr/include -I/usr/local/opt/boost/include -L/usr/local/opt/gmp/lib -L/usr/local/opt/mpfr/lib -L/usr/local/opt/boost/lib -lgmp -lmpfr -lboost_system -lboost_thread-mt

% addpath(fullfile(currentDir, 'build'));
addpath('/Users/ciaratorguson/point_processing/build');
% pyenv('Version', '/Users/ciaratorguson/myenv2/bin/python', 'ExecutionMode', 'InProcess');

% Create figure
f = figure('Name', 'Point Cloud Window', 'NumberTitle', 'off');
ax = axes(f);
set(ax, 'ButtonDownFcn', @(src, event) axes_ButtonDownFcn(src, event, f));
    function axes_ButtonDownFcn(~, event, f)
    % Check if pcData exists in f.UserDamex -v -largeArrayDims /Users/ciaratorguson/point_processing/test_includes.cpp -I/usr/local/Cellar/eigen/3.4.0_1/include/eigen3 -I/usr/local/opt/cgal/include -I/usr/local/opt/gmp/include -I/usr/local/opt/mpfr/include -I/usr/local/opt/boost/include -L/usr/local/opt/gmp/lib -L/usr/local/opt/mpfr/lib -L/usr/local/opt/boost/lib -lgmp -lmpfr -lboost_system -lboost_thread-mt -outdir /Users/ciaratorguson/point_processing/build -output PointProcessing
    
    if ~isfield(f.UserData, 'pcData') || isempty(f.UserData.pcData)
        % Display a warning message if pcData doesn't exist
        % warndlg('No point cloud data loaded!', 'Warning');
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
    if fileID == -1
        error('Failed to open the file.');
    end

    header = fgetl(fileID); % Read the header
    if ~strcmp(header, 'OFF')
        fclose(fileID);
        error('File is not in the OFF format.');
    end

    info = fscanf(fileID, '%d %d %d', [1 3]); % Read counts
    numVertices = info(1);
    numFaces = info(2);

    vertices = fscanf(fileID, '%f %f %f', [3 numVertices])'; % Read vertex data

    faces = zeros(numFaces, 4); % Assuming faces are quads for simplicity; adjust as needed
    for i = 1:numFaces
        faceInfo = fscanf(fileID, '%d', 1); % Read the number of vertices in the face
        faceVertices = fscanf(fileID, '%d', faceInfo); % Read vertex indices for the face
        faces(i, 1:length(faceVertices)) = faceVertices' + 1; % Adjust for MATLAB's 1-based indexing
    end

    fclose(fileID);
end



function [vertices, faces] = point_processing(pc)
    % Define the absolute path for the temporary .ply file
    tempFilePath = '/Users/ciaratorguson/point_processing/tempPointCloud.ply';
    
    % Convert point cloud data to a temporary .ply file
    pcwrite(pc, tempFilePath, 'Encoding', 'ASCII');
    
    % Log creation of the PLY file
    disp(['PLY file successfully created at ', tempFilePath]);
    
    % Optionally, read and display the first few lines of the PLY file for debugging
    fid = fopen(tempFilePath, 'r');
    if fid ~= -1
        disp('First 10 lines of the PLY file:');
        for i = 1:10 % Read and display the first 10 lines
            line = fgetl(fid);
            if ~ischar(line), break; end
            disp(line);
        end
        fclose(fid);
    else
        disp('Failed to open the PLY file.');
    end
    
    % Use the PointProcessing MEX function with the path to the .ply file
    try
        % This now expects a filename as output
        outputFile = PointProcessing(tempFilePath);
        disp(['Output file created: ', outputFile]);
    catch ME
        disp(['Error in PointProcessing: ', ME.message]);
        rethrow(ME);
    end
    
    % Define the full path to the output .off file
    outputMeshPath = fullfile('/Users/ciaratorguson/point_processing/build', outputFile);
    
    % Ensure the output file exists
    if ~exist(outputMeshPath, 'file')
        error('Mesh output file not found. Meshing may have failed.');
    end
    
    % Read the resulting mesh from the output .off file
    [vertices, faces] = read_off_file(outputMeshPath);
    
    % Optionally, delete the temporary .ply file
    delete(tempFilePath);
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
    [meshVertices, meshFaces] = point_processing(f.UserData.pcData);

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
