
function animateBike(state,p,motCommands, delta_offset, phi_offset)
%modified version of the file simulateBike

%this function is called at the end of the main navigation code to plot the
%results of our simulation of bicycle behavior; it will show an animation
%of the bicycle's behavior, as well as show a series of plots of the
%bicycle's state over time

close all;

%unpack state variables over time:
tarray = state(:,1);
x=state(:,2);
y=state(:,3);
z=zeros(size(x));
phi=state(:,4);
psi=state(:,5);
delta=state(:,6);
phi_dot=state(:,7);
v = state(:,8);

%%plot bicycle states
figure(1) 
hold on
subplot(2,2,1)
plot(tarray,phi,tarray, phi_offset);
title('lean vs. time');
xlabel('time (s)');
ylabel('phi');
legend("lean","desired lean");
subplot(2,2,2)
plot(tarray,phi_dot);
title('lean rate vs. time');
xlabel('time (s)');
ylabel('phi-dot');
subplot(2,2,3)
plot(tarray,delta);
hold on;
plot(tarray,delta_offset);
title('steer vs. time');
xlabel('time (s)');
ylabel('delta');
legend("steer", "desired steer")
subplot(2,2,4)
plot(tarray,motCommands);
title('steer rate vs. time');
xlabel('time (s)');
ylabel('delta-dot');

figure(2)
subplot(2,1,1)
plot(x,y);
axis equal
title('bicycle trajectory');
xlabel('x position');
ylabel('y position');
subplot(2,1,2)
psid = diff(psi)/diff(tarray);
plot(tarray(1:end-1),psid);
title('yaw dot vs. time');
xlabel('time (s)');
ylabel('yaw dot');

% figure(3)
% plot(tarray,navCommands);
% xlabel('time');
% ylabel('steer commmands');
% title('target steer angle vs. time');

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

[COG_hand,SH_hand,CPfw_hand,CPrw_hand]=drawBike(x(1),y(1),z(1),psi(1),phi(1),delta(1), p);
trajectory=plot3(x(1),y(1),z(1),'c');
axis([(x(1)-3) (x(1)+3) (y(1)-3) (y(1)+3) 0 (x(1)+3)])

pause(p.pause)

for i=1:length(x)
  if ~ishandle(animationFig)
      break;
  end
%delete(COG_hand,SH_hand,CPfw_hand,CPrw_hand,trajectory)    
delete(COG_hand)
delete(SH_hand)
delete(CPfw_hand)
delete(CPrw_hand)
delete(trajectory)
    
[COG_hand,SH_hand,CPfw_hand,CPrw_hand]=drawBike(x(i),y(i),z(i),psi(i),phi(i),delta(i),p);
trajectory=plot3(x(1:i),y(1:i),z(1:i),'c');
hold on;
plot(x(1:i),y(1:i),'r');

axis([(x(i)-2) (x(i)+2) (y(i)-2) (y(i)+2) 0 4]);

pause(p.pause)
end

end