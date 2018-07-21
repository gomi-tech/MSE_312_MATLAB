clear all
syms t s

% Parameters
Ra = 4.33; % Resistance (Ohm)
La = 0.00234; % Inductance (H)
Jm = 0.0000016; % Motor Inertia (kgm2)
Bm = 0.0000014; % Viscous Damping (Nms)
Ki = 0.0218; % Kt - Torque Constant (Nm/A)
Kb = 0.0218; % Ke - Back EMF Constant (V/rad/s)
Kt = 0.0286; % Sensor gain

set_pos = 45/s;
set_sp = 100/s;
TL = 0.0069/s; % load torque; TL = D/s; D = disturbance 
               % stall torque of motor is 0.3
               % continuous torque of motor is 0.20208 Nm
t_plot = 0:0.01:0.5;

%% Transfer functions
% Speed transfer function with load
sp_tf_WL = (Ki/(La*s+Ra)-TL)*(1/(Jm*s+Bm))/(1 + Kb*(Ki/(La*s+Ra)-TL)*(1/(Jm*s+Bm)));
%  = (set_sp*GL)/(1 + GL*Kt);

% Speed transfer function no load
sp_tf_NL = (Ki/(La*s+Ra)*(1/(Jm*s+Bm)))/(1 + Kb*(Ki/(La*s+Ra)*(1/(Jm*s+Bm))));
%  = (set_sp*G)/(1+G*Kt);

% Position transfer function with load
pos_tf_WL = (1/s)*((Ki/(La*s+Ra))-TL)*(1/(Jm*s+Bm)) / (1+(Kb*((Ki/(La*s+Ra))-TL)*(1/(Jm*s+Bm))));

% Position transfer function no load
pos_tf_NL = ((1/s)*((Ki/(La*s+Ra)))*(1/(Jm*s+Bm))) / (1+(Kb*((Ki/(La*s+Ra)))*(1/(Jm*s+Bm))));

%% P Controller 
% % Speed with load
% Kp_SP_WL = 0.05;
% P_sp_tf_WL = set_sp * (Kp_SP_WL*sp_tf_WL)/(1 + sp_tf_WL*Kt);
% P_sp_tf_WL_tolaplace = ilaplace(P_sp_tf_WL,s,t);
% P = zeros(1,51);
% i = 1;
% 
% for time = 0:0.01:0.5
% P(i) = vpa(subs(P_sp_tf_WL_tolaplace,t,time));
% i = i + 1;
% end
% 
% figure('Name','P - Speed with Load')
% plot(t_plot,P)
% title('P - Speed with Load')
% xlabel('Time (s)')
% ylabel('Speed (rad/s)')
% 
% % Speed no load
% Kp_SP_NL = 0.05;
% P_sp_tf_NL = set_sp * (Kp_SP_NL*sp_tf_NL)/(1 + sp_tf_NL*Kt);
% P_sp_tf_NL_tolaplace = ilaplace(P_sp_tf_NL,s,t);
% P = zeros(1,51);
% i = 1;
% 
% for time = 0:0.01:0.5
% P(i) = vpa(subs(P_sp_tf_NL_tolaplace,t,time));
% i = i + 1;
% end
% 
% figure('Name','P - Position with No Load')
% plot(t_plot,P)
% title('P - Position with No Load')
% xlabel('Time (s)')
% ylabel('Speed (rad/s)')


% Position with load
Kp_P_WL = 20;
P_pos_tf_WL = set_pos * ((Kt * Kp_P_WL * pos_tf_WL) / (1 + Kt * Kp_P_WL * pos_tf_WL));
P_pos_tf_WL_tolaplace = ilaplace(P_pos_tf_WL,s,t);

P = zeros(1,51);
i = 1;
for time = 0:0.01:0.5
P(i) = vpa(subs(P_pos_tf_WL_tolaplace,t,time));
i = i + 1;
end

figure('Name','P - Position with Load')
plot(t_plot,P)
title('P - Position with Load')
xlabel('Time (s)')
ylabel('Position (degrees)')


% Position no load
Kp_P_NL = 20;
P_pos_tf_NL = set_pos * ((Kt * Kp_P_NL * pos_tf_NL) / ( 1 + Kt * Kp_P_NL * pos_tf_NL));
P_pos_tf_NL_tolaplace = ilaplace(P_pos_tf_NL,s,t);
P = zeros(1,51);
i = 1;

for time = 0:0.01:0.5
P(i) = vpa(subs(P_pos_tf_NL_tolaplace,t,time));
i = i + 1;
end

figure('Name','P - Position with No Load')
plot(t_plot,P)
title('P - Position with No Load')
xlabel('Time (s)')
ylabel('Position (degrees)')

%% PD Controller 
% % Speed with load
% Kp_SPD_WL = 39;
% Kd_SPD_WL = 0.3;
% Gc_SPD = Kp_SPD_WL + Kd_SPD_WL*(s);
% P_spd_tf_WL = set_sp * ((Kt * Gc_SPD * sp_tf_WL)/( 1 + Kt * Gc_SPD * sp_tf_WL));
% P_spd_tf_WL_tolaplace = ilaplace(P_spd_tf_WL,s,t);
% P = zeros(1,51);
% i = 1;
% 
% for time = 0:0.01:0.5
% P(i) = vpa(subs(P_spd_tf_WL_tolaplace,t,time));
% i = i + 1;
% end
% 
% figure('Name','PD - Speed with Load')
% plot(t_plot,P)
% title('PD - Speed with Load')
% xlabel('Time (s)')
% ylabel('Speed (rad/s)')
% 
% % Speed no load
% Kp_S_NL = 0.05;
% P_sp_tf_NL = set_sp * (Kp_S_NL*sp_tf_NL)/(1 + sp_tf_NL*Kt);
% P_sp_tf_NL_tolaplace = ilaplace(P_sp_tf_NL,s,t);
% P = zeros(1,51);
% i = 1;
% 
% Kp_SPD_NL = 39;
% Kd_SPD_NL = 0.3;
% Gc_SPD = Kp_SPD_NL + Kd_SPD_NL*(s);
% P_spd_tf_NL = set_sp * ((Kt * Gc_SPD * sp_tf_NL)/( 1 + Kt * Gc_SPD * sp_tf_NL));
% P_spd_tf_NL_tolaplace = ilaplace(P_spd_tf_NL,s,t);
% P = zeros(1,51);
% i = 1;
% 
% for time = 0:0.01:0.5
% P(i) = vpa(subs(P_spd_tf_NL_tolaplace,t,time));
% i = i + 1;
% end
% 
% figure('Name','PD - Speed with No Load')
% plot(t_plot,P)
% title('PD - Speed with No Load')
% xlabel('Time (s)')
% ylabel('Speed (rad/s)')


% Position with load
Kp_PD_WL = 39;
Kd_PD_WL = 0.3;
Gc_PD = Kp_PD_WL + Kd_PD_WL*(s);
PD_pos_tf_WL = set_pos * ((Kt * Gc_PD * pos_tf_WL) / ( 1 + Kt * Gc_PD * pos_tf_WL));
PD_pos_tf_WL_tolaplace = ilaplace(PD_pos_tf_WL,s,t);
PD = zeros(1,51);
i = 1;

for time = 0:0.01:0.5
PD(i) = vpa(subs(PD_pos_tf_WL_tolaplace,t,time));
i = i + 1;
end

figure('Name','PD - Position with Load')
plot(t_plot,PD)
title('PD - Position with Load')


% Position no load
Kp_PD_NL = 39;
Kd_PD_NL = 0.3;
Gc_PD = Kp_PD_NL + Kd_PD_NL*(s);
PD_pos_tf_NL = set_pos * ((Kt * Gc_PD * pos_tf_NL) / ( 1 + Kt * Gc_PD * pos_tf_NL));
PD_pos_tf_NL_tolaplace = ilaplace(PD_pos_tf_NL,s,t);
PD = zeros(1,51);
i = 1;

for time = 0:0.01:0.5
PD(i) = vpa(subs(PD_pos_tf_NL_tolaplace,t,time));
i = i + 1;
end

figure('Name','PD - Position with No Load')
plot(t_plot,PD)
title('PD - Position with No Load')

%% PI Controller
% Speed with load


% Speed no load


% Position with load
Kp_PI_WL = 21;
Kd_PI_WL = 0.05;
Gc_PI = Kp_PI_WL + Kd_PI_WL/(s);
PI_pos_tf_WL = set_pos * ((Kt * Gc_PI * pos_tf_WL) / ( 1 + Kt * Gc_PI * pos_tf_WL));
PI_pos_tf_WL_tolaplace = ilaplace(PI_pos_tf_WL,s,t);
PI = zeros(1,51);
i = 1;

for time = 0:0.01:0.5
PI(i) = vpa(subs(PI_pos_tf_WL_tolaplace,t,time));
i = i + 1;
end

figure('Name','PI - Position with Load')
plot(t_plot,PI)
title('PI - Position with Load')


% Position no load
Kp_PI_NL = 21;
Kd_PI_NL = 0.05;
Gc_PI = Kp_PI_NL + Kd_PI_NL/(s);
PI_pos_tf_NL = set_pos * ((Kt * Gc_PI * pos_tf_NL) / ( 1 + Kt * Gc_PI * pos_tf_NL));
PI_pos_tf_NL_tolaplace = ilaplace(PI_pos_tf_NL,s,t);
PI = zeros(1,51);
i = 1;

for time = 0:0.01:0.5
PI(i) = vpa(subs(PI_pos_tf_NL_tolaplace,t,time));
i = i + 1;
end

figure('Name','PI - Position with No Load')
plot(t_plot,PI)
title('PI - Position with No Load')

%% PID Controller
% Speed with load


% Speed no load


% Position with load
Kp_PID_WL = 39; 
Kd_PID_WL = 0.02; 
Ki_PID_WL = 0.35; 
Gc_PID = Kp_PID_WL + Kd_PID_WL*(s) +Kd_PID_WL/(s);
PID_pos_tf_WL = set_pos * ((Kt * Gc_PID * pos_tf_WL) / ( 1 + Kt * Gc_PID * pos_tf_WL));
PID_pos_tf_WL_tolaplace = ilaplace(PID_pos_tf_WL,s,t);
PID = zeros(1,51);
i = 1;

for time = 0:0.01:0.5
PID(i) = vpa(subs(PID_pos_tf_WL_tolaplace,t,time));
i = i + 1;
end

figure('Name','PID - Position with Load')
plot(t_plot,PI)
title('PID - Position with Load')


% Position no load
Kp_PID_NL = 39; 
Kd_PID_NL = 0.02; 
Ki_PID_NL = 0.35; 
Gc_PID = Kp_PID_NL + Kd_PID_NL*(s) +Kd_PID_NL/(s);
PID_pos_tf_NL = set_pos * ((Kt * Gc_PID * pos_tf_NL) / ( 1 + Kt * Gc_PID * pos_tf_NL));
PID_pos_tf_NL_tolaplace = ilaplace(PID_pos_tf_NL,s,t);
PID = zeros(1,51);
n_PID = 1;

for time = 0:0.01:0.5
PID(i) = vpa(subs(PID_pos_tf_NL_tolaplace,t,time));
i = i + 1;
end

figure('Name','PID - Position with No Load')
plot(t_plot,PI)
title('PID - Position with No Load')
