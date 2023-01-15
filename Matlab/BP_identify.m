% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OMAR MEEBED                   %
% GM-MA3, EPFL                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Identify a frequency model data for Ball and Plate system
clear; close all; clc;

%% Load Data

% Read data
fileID = fopen('BP_data/logsXS3.bin');
data = fread(fileID,'double');
fclose(fileID);

N = length(data)/6;
data = reshape(data,6,N);
Ts = 0.017; % Sampling time

%% Preprocessing of data

% Trim data to region of interest
start = 1001;
stop = 2240;
N = stop-start+1;

y_1 = data(1,start:stop); % Output X coordinate of ball
y_2 = data(2,start:stop); % Output Y coordinate of ball
r_1 = data(3,start:stop); % Reference signal on X
r_2 = data(4,start:stop); % Reference signal on Y
u_1 = data(5,start:stop); % Input angle alpha
u_2 = data(6,start:stop); % Input angle beta

% Check periodity of data
[Ruu,h1] = xcorr(r_1(1,:),r_1(1,:),'unbiased');

n = 8; % Period found (Change accordingly)

% Remove first 2 periods
y_1 = y_1(2*N/n+1:end); 
y_2 = y_2(2*N/n+1:end); 
r_1 = r_1(2*N/n+1:end); 
r_2 = r_2(2*N/n+1:end); 
u_1 = u_1(2*N/n+1:end); 
u_2 = u_2(2*N/n+1:end);

% Detrend signal
y_f_1 = detrend(y_1); 
y_f_2 = detrend(y_2);

%% Spectral Analysis

% Generate range of frequencies to be identified
freq_nyquist = pi/Ts/5;
freqlist = freq_nyquist/N*n:freq_nyquist/N*n:freq_nyquist;

% Frequency response from r to y
ZT_1 = iddata(y_f_1',r_1',Ts);
ZT_2 = iddata(y_f_2',r_1',Ts);
TS11_3 = spa(ZT_1,N/n,freqlist); % X coordinate
TS12_3 = spa(ZT_2,N/n,freqlist); % Y coordinate

% Frequency response from u to y
ZU_1 = iddata(u_1',r_1',Ts);
ZU_2 = iddata(u_2',r_1',Ts);
US11_3 = spa(ZU_1,N/n,freqlist); % X coordinate
US12_3 = spa(ZU_2,N/n,freqlist); % Y coordinate

figure; bode(TS11_3); title('T11')
figure; bode(TS12_3); title('T12')
figure; bode(US11_3); title('U11')
figure; bode(US12_3); title('U12')

%% Save transfer functions T and U
filename = 'BP_TFs/GXS3.mat';
save(filename,'TS1_3','US1_3','TS2_3','US2_3');
