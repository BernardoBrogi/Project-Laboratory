close all
clear all

A=load('DemonstratedTrajectory_sim_to_real.txt');
B= load('ExecutedTrajectory.txt');

t=B(:,1);
X=B(:,5);
Y=B(:,9);
Z=B(:,13);

figure(1)
plot(A(:,1))
hold on
plot(X,'--' )
title('Positions along x-axis')
xlabel('time (ms)')
ylabel('position')

figure(2)
plot(A(:,2))
hold on
plot(Y,'--' )
title('Positions along y-axis')
xlabel('time (ms)')
ylabel('position')

figure(3)
plot(A(:,3))
hold on
plot(Z,'--' )
title('Positions along z-axis')
xlabel('time (ms)')
ylabel('position')




