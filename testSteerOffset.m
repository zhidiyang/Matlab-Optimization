%Written by Dylan Meehan (dem292), S18
%Test given controllers given a vector of commanded steer angles


%ICs
delta0 = 0;
phi0 = 0;
phi_dot0 = 0;
x = 0; 
y = 0;
psi0 = 0;
v = 3;  %m/s


%bike parameters: duplicated in runBicycleTest!
  p.g = 9.81; %acceleration due to gravity
  p.l = 1.02; %length of wheel base 

%controller gains:
%K= [11.1,2.4,-6.6]; %lqr
%K= [23,16,-2]; %large grid search optimization
%K= [71,21,-20]; %old gains from daniel (from nav sim)
K= [71,10,-20]; %other old gains

%nav instrcutions (as a vector of delta commands for each timestep
    %runBicycleTest calculates a lean offset to corresepond to a stable
    %turn at the desired Steer Angle
%make vector of delta commands



%delta_offset = makeStepOffset();
t = linspace(0,5,400);
delta_offset = (sin(t).*0.6)';

numTimeSteps = length(delta_offset);
phi_offset = v^2/p.l/p.g.*delta_offset; %steady state relation between phi & delta
%^this line is duplicated in runBicycleTest!!
    
delta0 = delta_offset(1);
phi0 = phi_offset(1);

[sucess,state] = runBicycleTest(x,y,v,delta0,phi0,phi_dot0,psi0, ...
    K,delta_offset,numTimeSteps, 1);

phi = state(:,4);
delta = state(:,6);

%score: judge navigation ability based on how well the bike can 
%    achieve the desired steer angle
nav_score = sum(sqrt((delta-delta_offset).^2))
   


function delta_offset = makeStepOffset()
    delta_offset = []; current_offset = 0;
    time_per_jump = 50; jump_per_timestep = 0.1;
    for i = 1:6
        current_offset = current_offset + jump_per_timestep;
        delta_offset = [delta_offset; ones(time_per_jump,1)*current_offset];
    end

    time_steps_per_step = 50; jump_per_timestep = -.5;
    for i = 1:3
        current_offset = current_offset + jump_per_timestep;
        delta_offset = [delta_offset; ones(time_per_jump,1)*current_offset];
    end
end
