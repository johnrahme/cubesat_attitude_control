function rpm = getRpm(deg)
Iw = 1.46*10^(-5);
Is = 1.67*10^(-3);

%Ta = 0.00196; % Torque in Nm
Ta = 0.004530672;
%Td = 0.00196; %Torque in Nm

Td = 0.03334261;

a = Ta/Iw; %Max angular acceleration

d = Td/Iw; %Max angular decelleration

rad = deg*(2*pi/360);

rpm = 60/pi*(rad*Is/(2*Iw)*a*d/(a+d)).^(0.5);

end