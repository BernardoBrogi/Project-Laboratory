close all
clear all

A = load('data_comm.txt');
B = load('data_mes.txt'); 
C = load('plan.txt');

%t=B(:,1)  ;
X_mes=B(:,1);
Y_mes=B(:,2);
Z_mes=B(:,3);

X_comm=A(:,1);
Y_comm=A(:,2);
Z_comm=A(:,3);

X_plan=C(:,1);
Y_plan=C(:,2);
Z_plan=C(:,3);

%% Plot of the differences between trajectory measured and trajectory commanded 

figure(1)
plot(A(:,1))
hold on
plot(X_mes,'--' )
title('Positions along x-axis')
xlabel('time (ms)')
ylabel('position')
xlim([0 5001])

figure(2)
plot(A(:,2))
hold on
plot(Y_mes,'--' )
title('Positions along y-axis')
xlabel('time (ms)')
ylabel('position')
xlim([0 5001])

figure(3)
plot(A(:,3))
hold on
plot(Z_mes,'--' )
title('Positions along z-axis')
xlabel('time (ms)')
ylabel('position')
xlim([0 5001])


%% Plot for comparison of the shape of the trajectory with LfD algorithm
figure(4)
subplot(1,2,1)
plot(A(:,3))
xlim([0 5001])
xlabel('time (ms)')
ylabel('position')
subplot(1,2,2)
plot(Z_plan,'--' )
%title('Positions along z-axis')
xlabel('time (ms)')
ylabel('position')



