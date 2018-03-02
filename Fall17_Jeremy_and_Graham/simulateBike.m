function simulateBike(state,tarray,x,y,p,navCommands,motCommands)

%this function is called at the end of the main navigation code to plot the
%results of our simulation of bicycle behavior; it will show an animation
%of the bicycle's behavior, as well as show a series of plots of the
%bicycle's state over time

close all;

%unpack state variables over time:

posx=state(1:(length(tarray)),1);
posy=state(1:(length(tarray)),2);
z=zeros(size(posx));
lean=state(1:(length(tarray)),3);
yaw=state(1:(length(tarray)),4);
steer=state(1:(length(tarray)),5);
leanRate=state(1:(length(tarray)),6);
velocity=state(1:(length(tarray)),7);


%%plot bicycle states
figure(1) 
hold on
subplot(2,2,1)
plot(tarray,lean);
title('lean vs. time');
xlabel('time');
ylabel('phi');
subplot(2,2,2)
plot(tarray,leanRate);
title('lean rate vs. time');
xlabel('time');
ylabel('phi-dot');
subplot(2,2,3)
plot(tarray,steer);
title('steer vs. time');
xlabel('time');
ylabel('delta');
subplot(2,2,4)
length(tarray)
length(motCommands)
plot(tarray,motCommands);
title('steer rate vs. time');
xlabel('time');
ylabel('delta-dot');

figure(2)
subplot(2,1,1)
plot(posx, posy);
hold on
plot(x,y,'LineWidth',1);
axis equal
title('bicycle trajectory');
xlabel('xposition');
ylabel('yposition');
legend('actual','desired','Location','Best');
subplot(2,1,2)
plot(tarray,yaw);
title('yaw vs. time');

figure(3)
plot(tarray,navCommands);
xlabel('time');
ylabel('steer commmands');
title('target steer angle vs. time');

%% animation for the bicycle--taken from Kate's animation, which is taken from Diego's animation
 %initialize figure for animation
   
   animationFig = figure(4);
   axis equal
   grid on
   hold on
   xlabel('x')
   ylabel('y')
   zlabel('z')
   az = 45;
   el = 30;
   view(az, el);

[COG_hand,SH_hand,CPfw_hand,CPrw_hand]=drawBike(posx(1),posy(1),z(1),yaw(1),lean(1),steer(1), p);
trajectory=plot3(posx(1),posy(1),z(1),'c');
axis([(posx(1)-3) (posx(1)+3) (posy(1)-3) (posy(1)+3) 0 (posx(1)+3)])

pause(p.pause)

for i=1:length(posx)
  if ~ishandle(animationFig)
      break;
  end
%delete(COG_hand,SH_hand,CPfw_hand,CPrw_hand,trajectory)    
delete(COG_hand)
delete(SH_hand)
delete(CPfw_hand)
delete(CPrw_hand)
delete(trajectory)
    
[COG_hand,SH_hand,CPfw_hand,CPrw_hand]=drawBike(posx(i),posy(i),z(i),yaw(i),lean(i),steer(i),p);
trajectory=plot3(posx(1:i),posy(1:i),z(1:i),'c');
hold on;
plot(x,y);

axis([(posx(i)-2) (posx(i)+2) (posy(i)-2) (posy(i)+2) 0 4]);

pause(p.pause)

end


end