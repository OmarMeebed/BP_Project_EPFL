% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OMAR MEEBED                   %
% GM-MA3, EPFL                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Save controller to the format used in LabVIEW code:
% Measurements = [error_X(k); error_Y(k); input(k-1); input(k-1);
%                   error_X(k-1); error_Y(k-1); input(k-2); input(k-2); 
%                   ... error_X(k-orderK); error_Y(k-orderK)]
clear; close; clc;

%% Load data

load BP_controllers/K11.mat
load BP_controllers/K22.mat

%% Convert to LabVIEW format

K11_X = K11_X/K11_Y(1);
K11_Y = K11_Y/K11_Y(1);
K22_X = K22_X/K22_Y(1);
K22_Y = K22_Y/K22_Y(1);

K11_Y = -K11_Y(2:end);
K22_Y = -K22_Y(2:end);

data = [K11_X(1) K22_X(1)];

for i=1:length(K11_X)-1
   data = [data K11_Y(i) K22_Y(i) K11_X(i+1) K22_X(i+1)]; 
end

%% Save controller

filename = 'BP_controllers/DDcontroller.bin';
fileID = fopen(filename,'w');
fwrite(fileID,data,'double');
fclose(fileID);