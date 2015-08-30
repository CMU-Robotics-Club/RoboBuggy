function trainSignDetector()
   load('stopSigns.mat');
   imDir = fullfile(matlabroot, 'toolbox', 'vision', 'visiondemos','stopSignImages');
   addpath(imDir);
   negativeFolder = fullfile(matlabroot, 'toolbox', 'vision','visiondemos', 'non_stop_signs');
   trainCascadeObjectDetector('stopSignDetector.xml', data, negativeFolder, 'FalseAlarmRate', 0.2, 'NumCascadeStages', 5);
end