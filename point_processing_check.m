% Define the path to the PLY file and the MEX file
plyFilePath = '/Users/ciaratorguson/point_processing/test.ply';
mexFilePath = '/Users/ciaratorguson/point_processing/build/PointProcessing.mexmaci64';

% Add the directory containing the MEX file to the MATLAB path
addpath(fileparts(mexFilePath));

% Call the MEX function with the PLY file path
try
    status = PointProcessing(plyFilePath);
    if status == 0
        disp('Success: PointProcessing function executed correctly.');
    else
        disp(['Error: PointProcessing function returned status ', num2str(status)]);
    end
catch ME
    disp('Error in executing PointProcessing MEX file:');
    disp(getReport(ME, 'extended'));
end
