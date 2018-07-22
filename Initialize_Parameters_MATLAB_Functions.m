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
b = K_e*nu/L;
c = V/L;
d = (K_t*nu)/( (J_L/E) + J_m*nu^2 );
e = (K_d*nu^2)/( (J_L/E) + J_m*nu^2 );


%For Calculating state space model A,B,C,D

A = [-a 0 -b;
     0 0 1;
     d 0 -e];
 
B = [c
     0
     0];

%% Make a separate A matrix for Load analysis
J_L = 1319858.29e-9; % Moment of Inertia of Load/Truss
d_l = (K_t*nu)/( (J_L/E) + J_m*nu^2 );
e_l = (K_d*nu^2)/( (J_L/E) + J_m*nu^2 );
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

%Calculate the poles and Zeros of the system.

b = [1.277e04];
a = [1, 1850, 7.374];
fvtool(b,a,'polezero');
[b,a] = eqtflength(b,a);
[z,p,k] = tf2zp(b,a);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          GAIN ANANLYSIS                 %
%*****************************************%
%*********     (P)  **********************%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5%%%
%------------------------------------------
%    GAIN ANANLYSIS - Position no Load    %
%------------------------------------------
G_pos = sys_pos;
k1 = 0.1;
k2 = 1;
k3 = 0.02;
T1 = feedback(G_pos*k1,1);
T2 = feedback(G_pos*k2,1);
T3 = feedback(G_pos*k3,1);

figure(2);
subplot(2,1,1);
step(T1,'b',T2,'r',T3,'g');
legend('k = 0.1','k = 1','k = 0.02');
title('Position-Gain Analysis no load');
hold on


%------------------------------------------
%    GAIN ANANLYSIS - Position w/ Load    %
%------------------------------------------
G_pos_L = sys_pos_L;
k1 = 0.1;
k2 = 1;
k3 = 0.02;
T1 = feedback(G_pos_L*k1,1);
T2 = feedback(G_pos_L*k2,1);
T3 = feedback(G_pos_L*k3,1);

subplot(2,1,2); 
step(T1,'b',T2,'r',T3,'g');
legend('k = 0.1','k = 1','k = 0.02');
title('Position-Gain Analysis w/ load');
hold off

%-----------------------------------------
%    GAIN ANANLYSIS - Velocity no Load    %
%-----------------------------------------

G_vel = sys_vel;
k1 = 0.1;
k2 = 1;
k3 = 0.02;
T1 = feedback(G_vel*k1,1);
T2 = feedback(G_vel*k2,1);
T3 = feedback(G_vel*k3,1);

figure(3);
subplot(2,1,1);
step(T1,'b',T2,'r',T3,'g');
legend('k = 0.1','k = 1','k = 0.02');
title('Velocity-Gain Analysis no load');
hold on

%-----------------------------------------
%    GAIN ANANLYSIS - Velocity w/ Load    %
%-----------------------------------------

G_vel_L = sys_vel_L;
k1 = 0.1;
k2 = 1;
k3 = 0.02;
T1 = feedback(G_vel_L*k1,1);
T2 = feedback(G_vel_L*k2,1);
T3 = feedback(G_vel_L*k3,1);


subplot(2,1,2); 
step(T1,'b',T2,'r',T3,'g');
legend('k = 0.1','k = 1','k = 0.02');
title('Velocity-Gain Analysis w/ load');
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         GAIN/INTEGRAL ANALYSIS          %
%*****************************************%
%******         PI      ******************%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5%%%
%---------------------------------------------------
%    GAIN/INTEGRAL ANANLYSIS - Position no Load    %
%---------------------------------------------------

G_pos = sys_pos;
k2 = 0.1;
ki1 = 10;
ki2 = 1;
ki3 = 0.1;
T1 = feedback(G_pos*(k2+(ki1/s)),1);
T2 = feedback(G_pos*(k2+(ki2/s)),1);
T3 = feedback(G_pos*(k2+(ki3/s)),1);

figure(4);
subplot(2,1,1);
step(T1,'b',T2,'r',T3,'g');
legend('ki = 10','ki = 1','ki = 0.1');
title('Position-PI Analysis no load');
hold on


%------------------------------------------
%    GAIN ANANLYSIS - Position w/ Load    %
%------------------------------------------
G_pos_L = sys_pos_L;
k2 = 0.1;
ki1 = 10;
ki2 = 1;
ki3 = 0.1;
T1 = feedback(G_pos_L*(k2+(ki1/s)),1);
T2 = feedback(G_pos_L*(k2+(ki2/s)),1);
T3 = feedback(G_pos_L*(k2+(ki3/s)),1);

subplot(2,1,2); 
step(T1,'b',T2,'r',T3,'g');
legend('k = 0.5','k = 1','k = 0.1');
title('Position-PI Analysis w/ load');
hold off

% %-----------------------------------------
% %    GAIN ANANLYSIS - Velocity no Load    %
% %-----------------------------------------
% 
% G_vel = sys_vel;
% k1 = 0.5;
% k2 = 1;
% k3 = 2;
% T1 = feedback(G_vel*k1,1);
% T2 = feedback(G_vel*k2,1);
% T3 = feedback(G_vel*k3,1);
% 
% figure(3);
% subplot(2,1,1);
% step(T1,'b',T2,'r',T3,'g');
% legend('k = 0.5','k = 1','k = 2');
% title('Velocity-Gain Analysis no load');
% hold on
% 
% %-----------------------------------------
% %    GAIN ANANLYSIS - Velocity w/ Load    %
% %-----------------------------------------
% 
% G_vel_L = sys_vel_L;
% k1 = 0.5;
% k2 = 1;
% k3 = 2;
% T1 = feedback(G_vel_L*k1,1);
% T2 = feedback(G_vel_L*k2,1);
% T3 = feedback(G_vel_L*k3,1);
% 
% 
% subplot(2,1,2); 
% step(T1,'b',T2,'r',T3,'g');
% legend('k = 0.5','k = 1','k = 2');
% title('Velocity-Gain Analysis w/ load');
% hold off
% 
% 
