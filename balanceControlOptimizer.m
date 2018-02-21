%This script is used to run tests on the bike to determine balance control.
%Our goal is to optimize the gains such that the bike can safely travel at 
%any speed and can recover from disturbances.

foldername = "GridSearchOptimization/";
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

%k3 should be the opposite sign of k1 and k2. GAINS
k1 = [70:71];
k2 = [10:11];
k3 = [-21:-20];  

result = zeros(length(k1)*length(k2)*length(k3),5);
trial = 1;

n1 = length(k1);
n2 = length(k2);
n3 = length(k3);

for a=1:n1
    for b=1:n2
        for c=1:n3

            K = [k1(a) k2(b) k3(c)];

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
            %result(trial,2) = sqrt(sum(phidot.^2)+sum(phi.^2)+sum(delta.^2));
            result(trial,2) = sqrt(sum(phi.^2));
            
%             %Scoring for Path location by distance from actual final waypoint
%             result(trial,3) = sqrt((x(end) - xb(end))^2+(y(end)-yb(end))^2);
            
            %Scoring for Path location based on yaw rate
            result(trial, 3) = sqrt(sum(psidot.^2));
            
            result(trial,4) = k1(a);
            result(trial,5) = k2(b);
            result(trial,6) = k3(c);
            trial = trial + 1;

        end
    end
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

%Find best test based on path score:
m = sortrows(m,3);
indm = find(m(:,1));
best2 = m(indm(1),:);

%Print best gains using balance score:
fprintf('Best gain values for v = %fm/s (balance score):\n',v)
fprintf('k1 = %d\nk2 = %d\nk3 = %d\n',best1(4),best1(5),best1(6))
fprintf('Balance Score = %f\n', best1(2))
fprintf('Path Score = %f\n', best1(3))
fprintf('success = %0.f\n\n', best1(1))

toc

fileID = fopen(path,'w');
fprintf(fileID, ' %s %s %s %s %s\n ',...
    ["ICs: ,","delta0="+num2str(delta0), ", phi0="+num2str(phi0),", phid="+num2str(phi_dot0),", Nonlinear EOM - Score = sqrt(phi^2 + phid^2 + delta^2)"]);
fprintf(fileID, '%s\n ',"success, balance_score, k1, k2, k3");
fclose(fileID);
dlmwrite(path,[success,balance_score,k_1,k_2,k_3], '-append');















