%Written by Dylan Meehan (dem292), S18
%Test given controllers given a vector of commanded steer angles


%ICs
delta0 = 0;
phi0 = 0;
phi_dot0 = 0;
x0 = 0; 
y0 = 0;
psi0 = 0;
v0 = 3;  %m/s


%bike parameters: duplicated in runBicycleTest!
  p.g = 9.81; %acceleration due to gravity
  p.l = 1.02; %length of wheel base 

%controller gains:
K1= [11.1,2.4,-6.6]; %lqr
K2= [23,16,-2]; %large grid search optimization
K3= [71,21,-20]; %old gains from daniel (from nav sim)
K4= [71,10,-20]; %other old gains

controller = [K1; K2; K3; K4];
%nav instrcutions (as a vector of delta commands for each timestep
    %runBicycleTest calculates a lean offset to corresepond to a stable
    %turn at the desired Steer Angle
%make vector of delta commands



%delta_offset = makeStepOffset();
t = linspace(0,2*pi,400);
delta_offset = (sin(t).*pi./6)';

numTimeSteps = length(delta_offset);
phi_offset = v0^2/p.l/p.g.*delta_offset; %steady state relation between phi & delta
%^this line is duplicated in runBicycleTest!!

angle = figure('Name','Steer tests with sine input');
traj = figure('Name','Steer tests with sine input');figure(angle)
 
for i = 1:4
    K = controller(i,:);
    
    [sucess,state] = runBicycleTest(x0,y0,v0,delta0,phi0,phi_dot0,psi0, ...
    K,delta_offset,numTimeSteps, 0);

    %score: judge navigation ability based on how well the bike can 
    %    achieve the desired steer angle

    figure(angle)
    times = state(:,1);
    phi = state(:,4);
    delta = state(:,6);
    phidot = state(:,7);
    deltadot = diff(delta)./diff(times);
    psi = state(:,5);
    x = state(:,2);
    y = state(:,3);

    subplot(2,2,1)
    plot(times,phi);
    hold on
    title('lean vs. time');
    xlabel('time (s)');
    ylabel('phi');
    subplot(2,2,2)
    hold on
    plot(times,phidot);
    title('lean rate vs. time');
    xlabel('time (s)');
    ylabel('phi-dot');
    subplot(2,2,3)
    hold on
    plot(times,delta);
    title('steer vs. time');
    xlabel('time (s)');
    ylabel('delta');
    subplot(2,2,4)
    hold on
    plot(times(1:end-1),deltadot);
    title('steer rate vs. time');
    xlabel('time (s)');
    ylabel('deltadot');
    
    figure(traj)
    subplot(2,1,1)
    hold on
    plot(x,y);
    axis equal
    title('bicycle trajectory');
    xlabel('x position');
    ylabel('y position');
    subplot(2,1,2)
    hold on
    psid = diff(psi)/diff(times);
    plot(times(1:end-1),psid);
    title('yaw dot vs. time');
    xlabel('time (s)');
    ylabel('yaw dot');
    
    
    nav_score = sqrt(sum((delta-delta_offset).^2));
    disp(K); disp(nav_score)
    
end

figure(angle)
subplot(2,2,1)
plot(times,phi_offset);
hold on
subplot(2,2,3)
plot(times,delta_offset);
hold on
subplot(2,2,1)
legend ( "lqr [11.1, 2.4, -6.6]", "grid [23, 16, -2]", "Nav [71, 21, -20]",...
    "other [71,10,-20]", "desired")
shg

figure(traj)
legend ("lqr [11.1, 2.4, -6.6]", "grid [23, 16, -2]", "Nav [71, 21, -20]",...
    "other [71,10,-20]")
shg



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
