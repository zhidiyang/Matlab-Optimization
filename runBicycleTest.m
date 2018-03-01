function [success, allStates] = runBicycleTest(x0,y0,v0,delta0,...
                                phi0,phi_dot0, psi0, K, delta_offset, graph)
%function to simulate a bicycle for given intial conditions and gains
%   similiar to the function mainNavigation developed in previous semesters
%   written by Dylan Meehan (dem292), Spring 2018

% x0,y0: intial location
% v0: initial speed
% delta0: intial steer angle
% phi0: intial lean angle
% phi_dot0: intial derivative of lean angle
% psi0: intial yaw angle (heading)

%define parameters
   p.g = 9.81; %acceleration due to gravity
   p.l = 1.02; %length of wheel base 
   p.b = 0.33; %distance from rear wheel to COM projected onto ground
   p.h = 0.516; %height of COM in point mass model
    % h is not the same as the height of COM of the bicycle, h is
    % calculated to place the center of mass so that the point
    % mass model and the real bicycle fall with the same falling frequency.
    % see ABT Fall 2017 report for further discussion.
   p.c = 0;   %trail

%calculate lean correction for desired steer
phi_offset = v0^2/p.l/p.g*delta_offset; %steady state relation between phi & delta
    %assumes constant velocity (v0)


% K: vector of gains (k1, k2, k3) 
    %reasonable gains are [70, 10, -20]
% graph: 1=  draws graph, 0 =does not
   
   p.pause = 1/50; %time to pause for animation
   
%parameters
timestep = 1/50;  %seconds
numTimeSteps = 400;

   
%initialize arrays of state and actuator data
count = 0;
success = 1;
tstart = 0;
currentState = [tstart x0 y0 phi0 psi0 delta0 phi_dot0 v0];
allStates = [currentState]; %keep track of the state at each timestep
motCommands = [0]; %command steer angle rate (delta dot) of front motor
 
 while (count < numTimeSteps)    
   [zdot, u] = rhs(currentState,p,K, delta_offset, phi_offset); %calculate derivatives based on EOM
    %also calculate u=delta_dot, the desired steer rate the keep balance
 
   % If the lean angle is too high, the test should count as a failure,
   % ie, the bicycle falls over
   phi = currentState(4);
   if abs(phi)>=pi/4
       fprintf('Bike has Fallen; Test Failure\n')
       success = 0; %failure
       break;
   end
   
   %update state
   previousState = currentState;
   currentState(1,1) = previousState(1,1) + timestep; %update time
   currentState(1,2:end) = previousState(1,2:end) + zdot*timestep; %Euler integrate
   allStates = [allStates; currentState]; %record state
   motCommands = [motCommands;u]; %record motor commands
  
   count = count + 1;
 end
 
 if graph == 1
     clf
     animateBike(allStates,p,motCommands,delta_offset, phi_offset);
     %animateBike is a rename of simulateBike from older MATLAB versions
 end

end

