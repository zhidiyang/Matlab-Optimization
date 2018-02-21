%This script is used to run tests on the bike to determine balance control.
%Our goal is to optimize the gains such that the bike can safely travel at 
%any speed and can recover from disturbances.
clear

foldername = "LQRSearchOptimization/";
filename = 'small_test.csv';
path = char(foldername+filename);

%USE GAIN SCHEDULING TO DETERMINE OPTIMAL GAINS TO DRIVE AT SLOWEST
%POSSIBLE VELOCITY.
tic

%ICs
delta0 = 0;
phi0 = pi/6;
phi_dot0 = 0;
x = 0; 
y = 10;
psi0 = 0;
v = 3;  %m/s

%%%%%%%%%%    LQR %%%%%%%%%%%%%%
% constant matricies 
%from Shihao Wang's 2014 Report
    %define parameters  
    % !!!!!!!!!    THESE ARE DUPLICATED IN "runBicycleTest.m"    !!!!!!!!!
       g = 9.81; %acceleration due to gravity
       l = 1.02; %length of wheel base 
       b = 0.33; %distance from rear wheel to COM projected onto ground
       h = 0.516; %height of COM in point mass model
        % h is not the same as the height of COM of the bicycle, h is
        % calculated to place the center of mass so that the point
        % mass model and the real bicycle fall with the same falling frequency.
        % see ABT Fall 2017 report for further discussion.
       c = 0;   %trail

A = [   0       1       0
       g/h      0  -v^2/(h*l)
        0       0       0     ];
B = [   0  -b*v/(h*l)   1]';

%t = linspace(0.01,1,100);
t = logspace(-2,2,100);
result = zeros(length(t),5);
trial = 1;

for t = t

    Q = t*[0 0 0; 0 0 0; 0 0 1];
    R = [1];

    [K,S,e] = lqr(A,B,Q,R);
    K = -1*K; %get K from lqr controller to match sign convention


    [success, state] = runBicycleTest(x,y,v,delta0,phi0,phi_dot0,psi0,K,0,0);  
    phi = abs(state(:,3));
    delta = abs(state(:,5));
    phidot = abs(state(:,6));
    psidot = abs(state(:,8));
    xb = state(:,1);
    yb = state(:,2);

    % Was run Successful?
    result(trial,1) = success;
    
    %Scoring for Balance (want lean rate to converge to 0)
    result(trial,2) = sqrt(sum(phidot.^2)+sum(phi.^2)+sum(delta.^2));
    %result(trial,2) = sqrt(sum(phi.^2));

    result(trial,4) = K(1);
    result(trial,5) = K(2);
    result(trial,6) = K(3);
    trial = trial + 1;
    
end
    

success = result(:,1);
balance_score = result(:,2);
path_score = result(:,3);
k_1 = result(:,4);
k_2 = result(:,5);
k_3 = result(:,6);

T = table(success,balance_score,path_score,k_1,k_2,k_3)
m = table2array(T);

%Find best test based on balance score:
m = sortrows(m,2);
indm = find(m(:,1));  %filters out failures
best1 = m(indm(1),:); 

%Print best gains using balance score:
fprintf('Best gain values for v = %fm/s (balance score):\n',v)
fprintf('k1 = %d\nk2 = %d\nk3 = %d\n',best1(4),best1(5),best1(6))
fprintf('Balance Score = %f\n', best1(2))
fprintf('Path Score = %f\n', best1(3))
fprintf('success = %0.f\n\n', best1(1))

toc

% fileID = fopen(path,'w');
% fprintf(fileID, ' %s %s %s %s %s\n ',...
%     ["ICs: ,","delta0="+num2str(delta0), ", phi0="+num2str(phi0),", phid="+num2str(phi_dot0),", Nonlinear EOM"]);
% fprintf(fileID, '%s\n ',"success, balance_score, k1, k2, k3");
% fclose(fileID);
% dlmwrite(path,[success,balance_score,k_1,k_2,k_3], '-append');















