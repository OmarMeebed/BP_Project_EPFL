% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OMAR MEEBED                   %
% GM-MA3, EPFL                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Merge frequency response data to get MIMO TF
clear; close all; clc;

%% Load Data

load BP_TFs/GX1.mat 
load BP_TFs/GX2.mat 
load BP_TFs/GX3.mat
load BP_TFs/GY1.mat 
load BP_TFs/GY2.mat
load BP_TFs/GY3.mat 
load BP_TFs/GXS1.mat 
load BP_TFs/GXS2.mat 
load BP_TFs/GXS3.mat
load BP_TFs/GYS1.mat 
load BP_TFs/GYS2.mat
load BP_TFs/GYS3.mat 

%% Combine frequency response data

T11_1 = frd(cat(3, TS11_1.ResponseData, T11_1.ResponseData(7:end)), [TS11_1.Frequency; T11_1.Frequency(7:end)], 0.017);
T11_2 = frd(cat(3, TS11_2.ResponseData, T11_2.ResponseData(7:end)), [TS11_2.Frequency; T11_2.Frequency(7:end)], 0.017);
T11_3 = frd(cat(3, TS11_3.ResponseData, T11_3.ResponseData(7:end)), [TS11_3.Frequency; T11_3.Frequency(7:end)], 0.017);

T12_1 = frd(cat(3, TS12_1.ResponseData, T12_1.ResponseData(7:end)), [TS12_1.Frequency; T12_1.Frequency(7:end)], 0.017);
T12_2 = frd(cat(3, TS12_2.ResponseData, T12_2.ResponseData(7:end)), [TS12_2.Frequency; T12_2.Frequency(7:end)], 0.017);
T12_3 = frd(cat(3, TS12_3.ResponseData, T12_3.ResponseData(7:end)), [TS12_3.Frequency; T12_3.Frequency(7:end)], 0.017);

T21_1 = frd(cat(3, TS21_1.ResponseData, T21_1.ResponseData(7:end)), [TS21_1.Frequency; T21_1.Frequency(7:end)], 0.017);
T21_2 = frd(cat(3, TS21_2.ResponseData, T21_2.ResponseData(7:end)), [TS21_2.Frequency; T21_2.Frequency(7:end)], 0.017);
T21_3 = frd(cat(3, TS21_3.ResponseData, T21_3.ResponseData(7:end)), [TS21_3.Frequency; T21_3.Frequency(7:end)], 0.017);

T22_1 = frd(cat(3, TS22_1.ResponseData, T22_1.ResponseData(7:end)), [TS22_1.Frequency; T22_1.Frequency(7:end)], 0.017);
T22_2 = frd(cat(3, TS22_2.ResponseData, T22_2.ResponseData(7:end)), [TS22_2.Frequency; T22_2.Frequency(7:end)], 0.017);
T22_3 = frd(cat(3, TS22_3.ResponseData, T22_3.ResponseData(7:end)), [TS22_3.Frequency; T22_3.Frequency(7:end)], 0.017);

U11_1 = frd(cat(3, US11_1.ResponseData, U11_1.ResponseData(7:end)), [US11_1.Frequency; U11_1.Frequency(7:end)], 0.017);
U11_2 = frd(cat(3, US11_2.ResponseData, U11_2.ResponseData(7:end)), [US11_2.Frequency; U11_2.Frequency(7:end)], 0.017);
U11_3 = frd(cat(3, US11_3.ResponseData, U11_3.ResponseData(7:end)), [US11_3.Frequency; U11_3.Frequency(7:end)], 0.017);

U12_1 = frd(cat(3, US12_1.ResponseData, U12_1.ResponseData(7:end)), [US12_1.Frequency; U12_1.Frequency(7:end)], 0.017);
U12_2 = frd(cat(3, US12_2.ResponseData, U12_2.ResponseData(7:end)), [US12_2.Frequency; U12_2.Frequency(7:end)], 0.017);
U12_3 = frd(cat(3, US12_3.ResponseData, U12_3.ResponseData(7:end)), [US12_3.Frequency; U12_3.Frequency(7:end)], 0.017);

U21_1 = frd(cat(3, US21_1.ResponseData, U21_1.ResponseData(7:end)), [US21_1.Frequency; U21_1.Frequency(7:end)], 0.017);
U21_2 = frd(cat(3, US21_2.ResponseData, U21_2.ResponseData(7:end)), [US21_2.Frequency; U21_2.Frequency(7:end)], 0.017);
U21_3 = frd(cat(3, US21_3.ResponseData, U21_3.ResponseData(7:end)), [US21_3.Frequency; U21_3.Frequency(7:end)], 0.017);

U22_1 = frd(cat(3, US22_1.ResponseData, U22_1.ResponseData(7:end)), [US22_1.Frequency; U22_1.Frequency(7:end)], 0.017);
U22_2 = frd(cat(3, US22_2.ResponseData, U22_2.ResponseData(7:end)), [US22_2.Frequency; U22_2.Frequency(7:end)], 0.017);
U22_3 = frd(cat(3, US22_3.ResponseData, U22_3.ResponseData(7:end)), [US22_3.Frequency; U22_3.Frequency(7:end)], 0.017);

%% Construct MIMO TFs
T_1 = [T11_1, T12_1; T21_1, T22_1];
U_1 = [U11_1, U12_1; U21_1, U22_1];

T_2 = [T11_2, T12_2; T21_2, T22_2];
U_2 = [U11_2, U12_2; U21_2, U22_2];

T_3 = [T11_3, T12_3; T21_3, T22_3];
U_3 = [U11_3, U12_3; U21_3, U22_3];

G_1 = T_1/U_1;
G_2 = T_2/U_2;
G_3 = T_3/U_3;

G_BP = stack(1,G_1,G_2,G_3);
T = stack(1,T_1,T_2,T_3);
U = stack(1,U_1,U_2,U_3);
%% Plot G with multimodal uncertainty

G_BP.OutputName = {'X','Y'};
G_BP.InputName = {'α','ß'};
figure;
bodemag(G_BP(:,:,1),'g',G_BP(:,:,2),'r',G_BP(:,:,3),'b')
title('System Frequency Response')
legend('G1','G2','G3')
grid
%% Choose and plot G_nominal

Gnom = G_3;
Gnom.OutputName = {'X','Y'};
Gnom.InputName = {'α','ß'};
figure;
bodemag(Gnom)
title('System Frequency Response')
grid
%% Save G_nominal

filename = 'BP_TFs/G.mat';
save(filename,'Gnom','G_BP');