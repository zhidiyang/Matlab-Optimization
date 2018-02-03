%Dylan Meehan, Jan 2018, 
%Based on Shihao Wang 2014 Report

%%%%%%%%%%%%    physical constants         %%%%%%%%%%
l = 0.98; %wheel base (m)
b = 0.32; %distance from rear wheel to COM projected onto ground plane (m)
c = 0; %trail (m)
g = 9.81; %acceleration due to gravity (m/s^2)
h = 0.5156; %height of COM, based on Fall '17 calculation of the falling 
    % frequency of the bicycle. Not the actual COM since bicycle is not a 
    % point mass.
v = 3.5; %forward velocity (m/s). Equations linearized based on constant v


%%%%%%%%%    constant matricies            %%%%%%
%from Shihao Wang's 2014 Report
A = [   0       1       0
       g/h      0  -v^2/(h*l)
        0       0       0     ];
B = [   0  -b*v/(h*l)   1]';
 

%arbitrary, change these!
r1 = -9;
r2 = -10;
i1 = 5i;
ev = [r1+i1, r1-i1, r2]; %eigenvalues

K = place(A,B,ev) %find K matrix to yeild desired eigenvalues
%interpolate K matricies (of different speeds) to find full controller