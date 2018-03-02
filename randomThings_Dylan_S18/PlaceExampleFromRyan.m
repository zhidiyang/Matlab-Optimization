A = randn(4,4); %
B = randn(4,1);

r1 = -3;
r2 = -5;
i1 = 1i;
i2 = 1i; 

ev = [r1+i1, r1-i1, r2+i2, r2-i2]; %eigenvalues

K = place(A,B,ev) %calculate eigenvalues
%interpolate K matricies to find controller

eig(A-B*K) %Check eigenvalues