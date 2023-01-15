% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PHILIPPE SCHUCHERT            %
% SCI-STI-AK, EPFL              %
% philippe.schuchert@epfl.ch    %

% Modified by                   %

% OMAR MEEBED                   %
% GM-MA3, EPFL                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Example use of the datadriven command
clear; clc; close all

%% Things we know / have measured

Ts = 0.017;

load BP_TFs/G.mat
Gmm = G_BP(2,2,:);
G = Gmm(:,:,1);
% Here parameteric model is used, as the closed-loop will be simulated.
% Only the frequency response function (FRF) is needed for controller synthesis.

Kp = 25; Kd = 25; tau = 0.1;
z = tf('z',Ts);
K = Kp+2*Kd*(z-1)/((2*tau+Ts)*z-2*tau+Ts); % Inital controller that is known to be stabilizing

Sinit = feedback(1,G*K);

%disp(['Eigenvalues close-loop using initial controller: ', num2str(max(abs(eig(Sinit)))), ' (stable CL)']) % < 1 --> Stable Closed-loop

%%

% Initial controller of order 1, but we want a final controller of order 3:
% -> zero padding to get the correct order.
[num,den] = tfdata(K,'v'); % get numerator/denominator of PID controller
orderK = 5;
den(orderK+1) = 0; % zero padding
num(orderK+1) = 0; % zero padding

% Poles on unit cicle of the controller have to be in the fixed parts
Fy = 1; % no fixed parts in denominator.
den_new = den; %den_new = deconv(den,Fy); % den = conv(den_new,Fy).


%% SET-UP system info

[SYS, OBJ, CON, PAR] = datadriven.utils.emptyStruct(); % load empty structure

ctrl = struct('num',num,'den',den_new,'Ts',Ts,'Fx',1,'Fy',1); % assemble controller

SYS.controller = ctrl;
SYS.model = G; % Specify model(s)
SYS.W = G.Frequency; % specify frequency grid where problem is solved

%% Objective(s)
% See different fields of OBJ
wc = 1;
W1 = 1/c2d(makeweight(0.01,wc,2),Ts);
W4 = 40/c2d(makeweight(1.05,wc,0.01),Ts);
W3 = 1/c2d(300*tf(1),Ts);

OBJ.oinf.W1 = W1; % Only minimize || W1 S ||_\infty 
OBJ.oinf.W3 = W3;
OBJ.oinf.W4 = W4;

%% Constraints(s)
% See different fields of CON
[P,Info] = ucover(Gmm,G,45);

figure; bodemag((G-Gmm)/G,'b--',Info.W1,'r',{0.2,200});
xlim([0.3 200])
title('Relative error vs Magnitude of W2')
grid

CON.W2 = Info.W1;

%% Solve problem
% See different fields of PAR
PAR.tol = 1e-6; % stop when change in objective < 1e-4. 
PAR.maxIter = 40; % max Number of iterations

tic
[controller,obj] = datadriven.datadriven(SYS,OBJ,CON,PAR,'mosek');
toc

% Other solver can be used as last additional argument:
% [controller,obj] = datadriven(SYS,OBJ,CON,PAR,'sedumi'); to force YALMIP
% to use sedumi as solver.
% If mosek AND mosek Fusion are installed, you can use
% [controller,obj] = datadriven(SYS,OBJ,CON,PAR,'fusion');
% (much faster, no need for YALMIP as middle-man)

%% Analysis using optimal controller

Kdd = datadriven.utils.toTF(controller); % 
S = feedback(1,G*Kdd);
U = feedback(Kdd,G);
V = feedback(G,Kdd);

% Plot S
figure; bodemag(S,1/W1,'-.r')
legend('$\mathcal{S}$','$\frac{1}{W1}$','interpreter','LaTeX')
xlim([0.3 200])
title('\textbf{Closed loop Sensitivity} $\mathcal{S}$','interpreter','LaTeX')
grid

% Plot T
figure; bodemag(1-S,1/Info.W1,'-.r')
legend('$\mathcal{T}$','$\frac{1}{W2}$','interpreter','LaTeX')
xlim([0.3 200])
title('\textbf{Closed loop Sensitivity} $\mathcal{T}$','interpreter','LaTeX')
grid

% Plot U
figure; bodemag(U,1/W3,'-.r')
legend('$\mathcal{U}$','$\frac{1}{W3}$','interpreter','LaTeX')
xlim([0.3 200])
title('\textbf{Input Sensitivity} $\mathcal{U}$','interpreter','LaTeX')
grid

% Plot V
figure; bodemag(V,1/W4,'-.r')
legend('$\mathcal{V}$','$\frac{1}{W4}$','interpreter','LaTeX')
xlim([0.3 200])
title('\textbf{Disturbance} $\mathcal{V}$','interpreter','LaTeX')
grid

disp(['Hinf computed using trapz integration: ', num2str(obj.Hinf)])
disp(['Hinf true value: ', num2str(norm(S*W1,'inf'))])
% True Hinf value not accessible when using the FRF

%% Save controller
[num,den] = tfdata(Kdd,'v');

K11_X = num;
K11_Y = den;

filename = 'BP_controllers/K11.mat';
save(filename,'K11_X','K11_Y');