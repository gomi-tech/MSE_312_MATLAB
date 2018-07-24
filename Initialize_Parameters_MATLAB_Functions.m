%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Initialize Variables      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R = 4.33;       % Armature Resistance
L = 2.34e-3;    % Armature inductance           
V = 12;             
J_L = 0;
J_m = 1.6e-6;       % Moment of Inertia of Motor
E = 0.95;           % Efficiency of gear train 

K_e = 2.18e-2;      %
K_t = 2.18e-2;      %
K_d = 1.1e-4;       % Back EMF constant
nu = 1/6.3;         %


% Simplifying the A matrix (no load) here.
a = R/L;
b = K_e/(nu*L);
c = V/L;
d = (K_t*nu)/( (J_L*(nu^2))/E + J_m);
e = (K_d)/( (J_L*(nu^2))/E + J_m);


%For Calculating state space model A,B,C,D

A = [-a 0 -b;
     0 0 1;
     d 0 -e];
 
B = [c
     0
     0];

%% Make a separate A matrix for Load analysis
J_L = 6319858.29e-9; % Moment of Inertia of Load/Truss
d_l = (K_t*nu)/( (J_L*(nu^2))/E + J_m);
e_l = (K_d)/( (J_L*(nu^2))/E + J_m);
A_L = [-a 0 -b;
     0 0 1;
     d_l 0 -e_l];
 
%Form of the state space vector is [current ,position, velocity]
%Therefore to analyze position and velocity response of the system

C_pos = [0, 1, 0];
C_vel = [0, 0, 1];

D = [0];

%%Make a systems model for no load & load case
sysmodel_vel = ss(A,B,C_vel,D);
sysmodel_pos = ss(A,B,C_pos,D);
sysmodel_vel_L = ss(A_L,B,C_vel,D);
sysmodel_pos_L = ss(A_L,B,C_pos,D);


%Find the transfer function of the system using tf.
sys_vel = tf(sysmodel_vel);
sys_pos = tf(sysmodel_pos);
sys_vel_L = tf(sysmodel_vel_L);
sys_pos_L = tf(sysmodel_pos_L);

%%
%1111111111111111111111111111111111111111111111111111111111
%             PROPORTIONAL ANANLYSIS                      %    
%                                                         %
%1111111111111111111111111111111111111111111111111111111111

%------------------------------------------
%    GAIN ANANLYSIS - Position no Load    %
%------------------------------------------
G_pos = sys_pos;
k1 = 0.1;
k2 = 1;
k3 = 10;
T1 = feedback(G_pos*k1,1);
T2 = feedback(G_pos*k2,1);
T3 = feedback(G_pos*k3,1);

figure(2);
subplot(2,1,1);
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt);
legend('k = 0.1','k = 1','k = 10');
title('Position-Gain Analysis no load');
ylabel('postition (rads)');
hold on


%------------------------------------------
%    GAIN ANANLYSIS - Position w/ Load    %
%------------------------------------------
G_pos_L = sys_pos_L;
k1 = 0.1;
k2 = 1;
k3 = 10;
T1 = feedback(G_pos_L*k1,1);
T2 = feedback(G_pos_L*k2,1);
T3 = feedback(G_pos_L*k3,1);

subplot(2,1,2); 
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt);
legend('k = 0.1','k = 1','k = 10');
title('Position-Gain Analysis w/ load');
ylabel('postition (rads)');
hold off

%-----------------------------------------
%    GAIN ANANLYSIS - Velocity no Load    %
%-----------------------------------------

G_vel = sys_vel;
k1 = 0.1;
k2 = 1;
k3 = 10;
T1 = feedback(G_vel*k1,1);
T2 = feedback(G_vel*k2,1);
T3 = feedback(G_vel*k3,1);

figure(3);
subplot(2,1,1);
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt);
legend('k = 0.1','k = 1','k = 10');
title('Velocity-Gain Analysis no load');
ylabel('Veloctiy (rad/s)');
hold on

%-----------------------------------------
%    GAIN ANANLYSIS - Velocity w/ Load    %
%-----------------------------------------

G_vel_L = sys_vel_L;
k1 = 0.1;
k2 = 1;
k3 = 10;
T1 = feedback(G_vel_L*k1,1);
T2 = feedback(G_vel_L*k2,1);
T3 = feedback(G_vel_L*k3,1);


subplot(2,1,2); 
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt);
legend('k = 0.1','k = 1','k = 10');
title('Velocity-Gain Analysis w/ load');
ylabel('Veloctiy (rad/s)');
hold off


%%
%22222222222222222222222222222222222222222222222222222222222222222
%         PROPORTIONAL/INTEGRAL ANALYSIS                         %
%                                                                %
%22222222222222222222222222222222222222222222222222222222222222222

%----------------------------------------
%    PI ANANLYSIS - Position no Load    %
%---------------------------------------

G_pos = sys_pos;
s = tf('s');

k2 = 1;
ki1 = 0.1;
ki2 = 0.5;
ki3 = 1;
T1 = feedback(G_pos*(k2+(ki1/s)),1);
T2 = feedback(G_pos*(k2+(ki2/s)),1);
T3 = feedback(G_pos*(k2+(ki3/s)),1);

figure(4);
subplot(2,1,1);
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt);
legend('ki = 0.1','ki = 0.5','ki = 1');
title('Position-PI Analysis no load (Kp = 1)');
ylabel('postition (rads)');
hold on


%------------------------------------------
%    PI ANANLYSIS - Position w/ Load      %
%------------------------------------------
G_pos_L = sys_pos_L;
s = tf('s');

k2 = 1;
ki1 = 0.1;
ki2 = 0.5;
ki3 = 1;
T1 = feedback(G_pos_L*(k2+(ki1/s)),1);
T2 = feedback(G_pos_L*(k2+(ki2/s)),1);
T3 = feedback(G_pos_L*(k2+(ki3/s)),1);

subplot(2,1,2); 
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt);
legend('ki = 0.1','ki = 0.5','ki = 1');
title('Position-PI Analysis w/ load (Kp = 1)');
ylabel('postition (rads)');
hold off

%-----------------------------------------
%   PI ANANLYSIS - Velocity no Load      %
%-----------------------------------------

G_vel = sys_vel;
s = tf('s');

k2 = 1;
ki1 = 0.1;
ki2 = 0.5;
ki3 = 1;
T1 = feedback(G_pos_L*(k2+(ki1/s)),1);
T2 = feedback(G_pos_L*(k2+(ki2/s)),1);
T3 = feedback(G_pos_L*(k2+(ki3/s)),1);

figure(5);
subplot(2,1,1);
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt);
legend('ki = 0.1','ki = 0.5','ki = 1');
title('Velocity-PI Analysis no load (Kp = 1)');
ylabel('Veloctiy (rad/s)');
hold on

%-----------------------------------------
%    PI ANANLYSIS - Velocity w/ Load      %
%-----------------------------------------

G_vel_L = sys_vel_L;
s = tf('s');

k2 = 1;
ki1 = 0.1;
ki2 = 0.5;
ki3 = 1;
T1 = feedback(G_pos_L*(k2+(ki1/s)),1);
T2 = feedback(G_pos_L*(k2+(ki2/s)),1);
T3 = feedback(G_pos_L*(k2+(ki3/s)),1);


subplot(2,1,2); 
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt);
legend('ki = 0.1','ki = 0.5','ki = 1');
title('Velocity-PI Analysis w/ load (Kp = 1)');
ylabel('Veloctiy (rad/s)');
hold off


%%
%333333333333333333333333333333333333333333333333333333333333333333333333
%                 PROPORTIONAL DERIVATIVE ANALYSIS                       %  
%                                                                        %
%333333333333333333333333333333333333333333333333333333333333333333333333

%----------------------------------------
%    PD ANANLYSIS - Position no Load    %
%---------------------------------------

G_pos = sys_pos;
s = tf('s');

k2 = 1;
kd1 = 0.1;
kd2 = 1;
kd3 = 5;
T1 = feedback(G_pos*(k2+(kd1*s)),1);
T2 = feedback(G_pos*(k2+(kd2*s)),1);
T3 = feedback(G_pos*(k2+(kd3*s)),1);

figure(6);
subplot(2,1,1);
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt,1);
legend('kd = 0.1','kd = 1','kd = 5');
title('Position-PD Analysis no load (Kp = 1)');
ylabel('postition (rads)');
hold on


%------------------------------------------
%    PD ANANLYSIS - Position w/ Load      %
%------------------------------------------
G_pos_L = sys_pos_L;
s = tf('s');

k2 = 1;
kd1 = 0.1;
kd2 = 1;
kd3 = 5;
T1 = feedback(G_pos*(k2+(kd1*s)),1);
T2 = feedback(G_pos*(k2+(kd2*s)),1);
T3 = feedback(G_pos*(k2+(kd3*s)),1);

subplot(2,1,2); 
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt,1);
legend('kd = 0.1','kd = 1','kd = 5');
title('Position-PD Analysis w/ load (Kp = 1)');
ylabel('postition (rads)');
hold off

%-----------------------------------------
%   PD ANANLYSIS - Velocity no Load      %
%-----------------------------------------

G_vel = sys_vel;
s = tf('s');

k2 = 1;
kd1 = 0.1;
kd2 = 1;
kd3 = 5;
T1 = feedback(G_pos*(k2+(kd1*s)),1);
T2 = feedback(G_pos*(k2+(kd2*s)),1);
T3 = feedback(G_pos*(k2+(kd3*s)),1);


figure(7);
subplot(2,1,1);
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt,1);
legend('kd = 0.1','kd = 1','kd = 5');
title('Velocity-PD Analysis no load (Kp = 1)');
ylabel('Veloctiy (rad/s)');
hold on

%-----------------------------------------
%    PD ANANLYSIS - Velocity w/ Load      %
%-----------------------------------------

G_vel_L = sys_vel_L;
s = tf('s');

k2 = 1;
kd1 = 0.1;
kd2 = 1;
kd3 = 5;
T1 = feedback(G_pos*(k2+(kd1*s)),1);
T2 = feedback(G_pos*(k2+(kd2*s)),1);
T3 = feedback(G_pos*(k2+(kd3*s)),1);


subplot(2,1,2); 
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt,1);
legend('kd = 0.1','kd = 1','kd = 5');
title('Velocity-PD Analysis w/ load (Kp = 1)');
ylabel('Veloctiy (rad/s)');
hold off

%%
%4444444444444444444444444444444444444444444444444444444444444444444444444
%                 PROPORTIONAL DERIVATIVE ANALYSIS                       %  
%                                                                        %
%4444444444444444444444444444444444444444444444444444444444444444444444444

%---------------------------------------
%    PID ANANLYSIS - Position no Load   %
%---------------------------------------

G_pos = sys_pos;
s = tf('s');

k2 = 5;
ki1 = 0.05;
kd1 = 0.1;
kd2 = 0.001;
kd3 = 1;
T1 = feedback(G_pos*(k2+(ki1/s)+(kd1*s)),1);
T1 = feedback(G_pos*(k2+(ki1/s)+(kd2*s)),1);
T1 = feedback(G_pos*(k2+(ki1/s)+(kd3*s)),1);

figure(8);
subplot(2,1,1);
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt,1);
legend('kd = 0.05','kd = 0.1','kd = 0.001');
title('Position-PID Analysis no load (Kp = 5, Ki = 0.5)');
ylabel('postition (rads)');
hold on


%------------------------------------------
%    PID ANANLYSIS - Position w/ Load     %
%------------------------------------------
G_pos_L = sys_pos_L;
s = tf('s');

k2 = 5;
ki1 = 0.5;
kd1 = 0.1;
kd2 = 0.5;
kd3 = 1;
T1 = feedback(G_pos*(k2+(ki1/s)+(kd1*s)),1);
T1 = feedback(G_pos*(k2+(ki1/s)+(kd2*s)),1);
T1 = feedback(G_pos*(k2+(ki1/s)+(kd3*s)),1);


subplot(2,1,2); 
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt,1);
legend('kd = 0.5','kd = 0.1','kd = 1');
title('Position-PID Analysis no load (Kp = 5, Ki = 0.5)');
ylabel('Postition (rads)');
hold off

%-----------------------------------------
%   PID ANANLYSIS - Velocity no Load      %
%-----------------------------------------

G_vel = sys_vel;
s = tf('s');

k2 = 5;
ki1 = 0.5;
kd1 = 0.1;
kd2 = 0.5;
kd3 = 1;
T1 = feedback(G_pos*(k2+(ki1/s)+(kd1*s)),1);
T1 = feedback(G_pos*(k2+(ki1/s)+(kd2*s)),1);
T1 = feedback(G_pos*(k2+(ki1/s)+(kd3*s)),1);

figure(9);
subplot(2,1,1);
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt,1);
legend('kd = 0.5','kd = 0.1','kd = 1');
title('Position-PID Analysis no load (Kp = 5, Ki = 0.5)');
ylabel('Veloctiy (rad/s)');
hold on

%-----------------------------------------
%    PID ANANLYSIS - Velocity w/ Load    %
%-----------------------------------------

G_vel_L = sys_vel_L;
s = tf('s');

k2 = 5;
ki1 = 0.5;
kd1 = 0.1;
kd2 = 0.5;
kd3 = 1;
T1 = feedback(G_pos*(k2+(ki1/s)+(kd1*s)),1);
T1 = feedback(G_pos*(k2+(ki1/s)+(kd2*s)),1);
T1 = feedback(G_pos*(k2+(ki1/s)+(kd3*s)),1);


subplot(2,1,2); 
opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(T1,'b',T2,'r',T3,'g',opt,1);
legend('kd = 0.5','kd = 0.1','kd = 1');
title('Position-PID Analysis no load (Kp = 5, Ki = 0.5)');
ylabel('Veloctiy (rad/s)');
hold off



