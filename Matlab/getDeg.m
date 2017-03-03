function deg = getDeg(rpm)
Iw = 1.46*10^(-5);
Is = 1.67*10^(-3);

%Ta = 0.00196; % Torque in Nm
Ta = 0.004530672;
%Td = 0.00196; %Torque in Nm

Td = 0.03334261;

a = Ta/Iw; %Max angular acceleration

d = Td/Iw; %Max angular decelleration

aDeg = a*(360/(2*pi));

RpmMax = 4500;

Rpm = rpm; %If we don't want to reach maximum RPM change this

w = Rpm/60*2*pi;

wDeg = w*(360/(2*pi));

t1 = w/a; % Time to max rotation speed

t3 = w/d; % Time to stop from max Rotation speed

const1 = (Iw/Is*a/2);
const3 = (Iw/Is*d/2);

rad1 = t1.^2*const1; %Radians rotated when accelerating

rad3new = Iw/Is*t3.*(a*t1-d*t3/2);
rad3 = t3.^2.*const3; % Radians rotated when decellerating 

degTot = (rad1+rad3)*360/(2*pi);

deg = degTot;
end