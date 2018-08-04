function [] = plot_response_ki(plant,kp,kd,ki)
sys = plant;
s = tf('s');

R1 = feedback(plant*(kp + ki/s + kd),1);
R2 = feedback(plant*(kp + (ki*2)/s + kd),1);
R3 = feedback(plant*(kp + (ki*0.5)/s + kd),1);

opt = stepDataOptions('InputOffset',0,'StepAmplitude',3.14159/2);
step(R1,'b',R2,'r',R3,'g',opt);
legend('ki1 = '+ string(ki),'ki2 = '+ string(ki*2),'ki3 = '+ string(ki*0.5));
ylabel('postition (rads)');


end