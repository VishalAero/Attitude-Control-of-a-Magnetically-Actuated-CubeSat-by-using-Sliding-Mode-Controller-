close all; clear;
time=15;

% Initial states of the CubeSat
q0 = [deg2rad(57.2958),deg2rad(1.3),deg2rad(1.1),deg2rad(1)];     % quaternions
Omega0 = [0.002,  0.005,  0.001]; % Angular velocity vector in spacecraft body-fixed frame

%Combined Vector of all States
Vari_States = [q0'; Omega0'] ;

% Solving the CubeSat Dynamic Model states
[time,state] = ode45(@Model1, [0 time], Vari_States);

figure(1)
Rec_Omega = state(:,5:7);
Omega_x = Rec_Omega(:,1);
Omega_y = Rec_Omega(:,2);
Omega_z = Rec_Omega(:,3);
plot(time,Omega_x,'r',time,Omega_y,'b',time,Omega_z,'g')
ylim([(-0.15) (0.2)])
legend('\omega_x','\omega_y','\omega_z')
title('Spacecraft Body-Fixed Angular Rates')
xlabel('Time [sec]')
ylabel('Angular Rates [rad/sec]')


figure(2)
Rec_q = state(:,1:4);
q_m = Rec_q(:,1);
q_x = Rec_q(:,2);
q_y = Rec_q(:,3);
q_z = Rec_q(:,4);
plot(time,rad2deg(q_m),'k',time,rad2deg(q_x),'r',time,rad2deg(q_y),'b',time,rad2deg(q_z),'g')
ylim([-10 60])
grid on 
legend('Total Error','X error','Y error','Z error')
title('Pointing Angles of Body-Fixed Axes in Desired Reference Frame')
xlabel('Time [sec]')
ylabel('Pointing Angles [deg]')






