%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Initialize Variables      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R = 4.33;       % Armature Resistance
L = 2.34e-3;    % Armature inductance           
V = 12;             

J_L = 1319858.29e-9; % Moment of Inertia of Load   
J_m = 1.6e-6;       % Moment of Inertia of Motor
E = 0.95;           % Efficiency of gear train 

K_e = 2.18e-2;      %
K_t = 2.18e-2;      %
K_d = 1.1e-4;       % Back EMF constant
nu = 1/6.3;         %


% Simplifying the A matrix here.
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

 
%Form of the state space vector is [current ,position, velocity]
%Therefore to analyze position and velocity response of the system

C_pos = [0, 1, 0];
C_vel = [0, 0, 1];

D = [0];

sysmodel_vel = ss(A,B,C_vel,D);
sysmodel_pos = ss(A,B,C_pos,D);

%Find the transfer function of the system using tf.
sys_vel = tf(sysmodel_vel);
sys_pos = tf(sysmodel_pos);

%Calculate the poles and Zeros of the system.

b = [1.277e04]
a = [1, 1850, 7.374]
fvtool(b,a,'polezero')
[b,a] = eqtflength(b,a);
[z,p,k] = tf2zp(b,a)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    GAIN ANANLYSIS - Position     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s = tf('s');
G_pos = (1.277e04)/(s^3+1850*s^2+7.374*s);
k1 = 0.5;
k2 = 1;
k3 = 5;
T1 = feedback(G*k1,1);
T2 = feedback(G*k2,1);
T3 = feedback(G*k3,1);
figure(2)
step(T1,'b',T2,'r',T3,'g')
legend('k = 0.5','k = 1','k = 5')
title('Position-Gain Analysis');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    GAIN ANANLYSIS - Velocity     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s = tf('s');
G_vel = (1.277e04)/(s^2+1850*s+7.374);
k1 = 0.5;
k2 = 1;
k3 = 5;
T1 = feedback(G*k1,1);
T2 = feedback(G*k2,1);
T3 = feedback(G*k3,1);
figure(3)
step(T1,'b',T2,'r',T3,'g')
legend('k = 0.5','k = 1','k = 5')
title('Velocity-Gain Analysis');

% 
% pzmap(T)
% grid, axis([-2000 0 -1 1])
% 
% clf
% step(T)
% rlocus(G)
% grid

% %Simulate system response to step and impulse  using the transfer function
% %model.
% figure(3)
% subplot(2,1,1)
% step(sys)
% subplot(2,1,2)
% impulse(sys)



