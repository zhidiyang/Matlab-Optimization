function [success, state] = mainNavigation(x,y,v,delta0,phi0,ks,graph)
   %given a set of waypoints that the bicycle has to pass through, and a
   %set of velocities that we expect the bicycle to have in each of the
   %segments between the waypoints, we want to find out what command to
   %send to the steering motor so that the bicycle follows the path and
   %stays balanced
   
   %%x,y waypoint vector
   %v = speed is constant
   %delta initial steer
   %phi0  initial lean angle
   %graph: 1 = make graph
   
   %define parameters
   p.g = 9.81; p.l = 1.02; %length of wheel base %p.b=0.5; p.h=0.5;
   p.b = 0.3; p.h = 0.5; %h = height to COM, %b=??
   p.c = 0;   %trail is 0
   
   %gains
   p.k1 = ks(1);
   p.k2 = ks(2);
   p.k3 = ks(3);
   %k3 should have a different sign from other two gains.
   
   p.timestep = 1/60; %seconds
   p.pause = p.timestep; %pause for simulation, should not for gains opt
    
   %assume that we start out in the first segment
   %nav algorithm stuff
   tstep=p.timestep;
   tarray=[]; %store time at each timestep
   currentSegment=1;
   xC=x(currentSegment); yC=y(currentSegment); xD=x(currentSegment+1); yD=y(currentSegment+1);
   
   
   %once segment is chosen...
   %check whether bike has crossed boundary

%% Check continually whether the threshold within a segment has been passed; when that happens, the next segment becomes target
    
   %get information from sensors about state of bicycle
   xB=0;             %xB and yB represent the current position of the bike
   yB=0;
   thetaB = 0;       %thetaB=pi/8; xB,yB,thetaB taken from GPS data, same as yaw
   yaw0=0;           %heading information not actually necessary, but we keep it and treat the desired path (if straight line on x-axis) as psi=0
   phiDot0=0;
   initialConditions=[xB,yB,phi0,yaw0,delta0,phiDot0,v];
   state=initialConditions;
   
%% This bit cycles as the program updates the location and updates the controller to fix behavior of the bicycle

    %decides which segment it is in
    %how do we choose a segment?
    %how do we know if we are off course at all? Nav stuff
    [dis,ind]=min((xB-x).^2+(yB-y).^2);    %take the minimum distance between the location of the bicycle and any waypoint
    currentSegment=ind;
    xC=x(currentSegment); yC=y(currentSegment); xD=x(currentSegment+1); yD=y(currentSegment+1);
    
    
    %Set success = 1 to start. Assuming the test will be successful,
    %unless the bike crashes.
    success = 1;

    % Calculating the ideal distance the perfect path would travel based on
    % the give wayping
    wp_dist = 0;
    
    for i = 2:length(x)
        wp_dist = wp_dist + sqrt((x(i) - x(i-1))^2 + (y(i) - y(i-1))^2);
    end
    
    %Calculating actual distance bike travels
    actual_dist = 0;
    
    k=1;
    % Assigning loop conditions to stop stepping forward in time
    
    while actual_dist < wp_dist
        %"while the actual distance of the bike is less than the maximum x distance" 
        %(used for balance control in straight line).
                                     
    %while k <= 1000    %use for quick, constant integration time for all trials
    
    %while currentSegment<length(x)-1  
    
    %for k=1:length(x)/tstep
      time = (k-1)*tstep;
      tarray(k)=time;
      xB=state(k,1);
      yB=state(k,2);
      phiB=state(k,3);
      thetaB=state(k,4); %same thing as yaw
      v=state(k,7);
              
    %if boundary crossed, choose next segment
    threshold=1;    

    
    %figure out if we've crossed threshold; take the dot product between
    %the unit path and the distance from the bike to the origin of the
    %target segment
    pathLength=sqrt((xD-xC)^2+(yD-yC)^2);
    unitPath=[xD-xC,yD-yC,0]/pathLength;
    bikeLoc=[xB-xC,yB-yC,0];    
    dist=dot(unitPath,bikeLoc);
    
%     %if that threshold is passed, the bicycle should aim for the next
%     %segment
%     if dist>pathLength-threshold
%         currentSegment=currentSegment+1;
%         if currentSegment==length(x)
%             break
%             fprintf('reached end of path');
%         end
%         xC=x(currentSegment); yC=y(currentSegment); xD=x(currentSegment+1); yD=y(currentSegment+1);
%     end
    
    
   %update time in time array
   
   if round(time) == 200
       fprintf('k1 = %f ',ks(1));
       fprintf('k2 = %f ',ks(2));
       fprintf('k3 = %f\n',ks(3));
       return
   end

 
   %calculates desired steer, based on our target segment
   %steerD=findSteer(xB,yB,xC,yC,xD,yD,thetaB,v,p);
   steerD=0;
   navCommands(k)=steerD;
   
   %use all the sensor data and first feedback controller output (desired
   %steer angle) to find u, using the rhs function below:
   [zdot,u]=rhs(state(k,:),steerD,p,time,ks);
   motCommands(k)=u;
   
   %If the lean angle is too high, the test should count as a failure.
   %Set success = 0.
   if abs(phiB)>=pi/4
       fprintf('Bike has Fallen; Test Failure\n')
       success = 0;
       break;
   end
   
   %add the current state to the array of states that the bike has experienced
   %over the testing period
   % Use Euler to Solve for next state
   state(k+1,:)=state(k,:)+zdot*tstep;
   
   k=k+1;
   
   xB_new = state(k,1);
   yB_new = state(k,2);
   actual_dist = actual_dist + sqrt((xB_new - xB)^2 + (yB_new - yB)^2);
  end

   close all
   if graph == 1
       animateBike(state,tarray,x,y,p,navCommands,motCommands);
       %animateBike is a rename of simulateBike from older MATLAB versions
       %update xB,yB,other state variables (unpack state(end))
   end
end
   