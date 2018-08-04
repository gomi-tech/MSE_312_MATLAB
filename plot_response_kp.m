function [] = plot_response_kp(plant,kp,kd,ki)
sys = plant;
s = tf('s');

R1 = feedback(plant*(kp + ki/s + kd),1);
R2 = feedback(plant*((kp*2) + ki/s + kd),1);
R3 = feedback(plant*((kp*0.5)+ ki/s + kd),1);

opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(R1,'b',R2,'r',R3,'g',opt);
legend('kp1 = '+ string(kp),'kp2 = '+ string(kp*2),'kp3 = '+ string(kp*0.5));
ylabel('postition (rads)');


end
