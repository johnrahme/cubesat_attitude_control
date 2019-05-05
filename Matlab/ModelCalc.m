Iw = 1.46*10^(-5);
Is = 1.67*10^(-3);

%Ta = 0.00196; % Torque in Nm
Ta = 0.004530672;
%Td = 0.00196; %Torque in Nm

Td = 0.03334261;

a = Ta/Iw; %Max angular acceleration

d = Td/Iw; %Max angular decelleration

K = Iw*a/Iw;
sys = zpk([],[0 0 0],K);
sys
[C_pi,info] = pidtune(sys,'PID')