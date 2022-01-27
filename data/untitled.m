close all
clear all

A=load('data_comm.txt')
B= load('data_mes.txt') ;

%t=B(:,1)  ;
X_mes=B(:,1) ;
Y_mes=B(:,2) ;
Z_mes=B(:,3);

X_comm=A(:,1) ;
Y_comm=A(:,2) ;
Z_comm=A(:,3);


figure(1)
plot(A(:,1))
hold on
plot(X_mes,'--' )
title('Positions along x-axis')
xlabel('time (ms)')
ylabel('position')

figure(2)
plot(A(:,2))
hold on
plot(Y_mes,'--' )
title('Positions along y-axis')
xlabel('time (ms)')
ylabel('position')

figure(3)
plot(A(:,3))
hold on
plot(Z_mes,'--' )
title('Positions along z-axis')
xlabel('time (ms)')
ylabel('position')




