function output=Outerlooptest(input)

% Translational dynamic position and velocity states
x    = input(1);
xdot = input(2);
y    = input(3);
ydot = input(4);
z    = input(5);
zdot = input(6);

% Rotational dynamic angular position and velocity states
theta   = input(7);
thetadot= input(8);
phi     = input(9);
phidot  = input(10);
psi     = input(11);
psidot  = input(12);
t_sim   = input(13);

% constant parameters: mass, inertial values, gravity
m=1;
g=9.81;
Jx=0.2;
Jy=0.2;
Jz=0.2;

% PD controller control gains 
% Control gains for x
kpx=0.1; kdx=0.09;
% Control gains for y
kpy=0.1; kdy=0.09;
% Control gains for z
kpz=1;   kdz=0.7;
% Control gains for roll
kptheta=3; kdtheta=1;
% Control gains for pitch
kpphi=3;   kdphi=1;
% Control gains for yaw
kppsi=3;   kdpsi=1;

% Desired position and velocity for x and y
xd=5;
xdotd=0;
yd=-6;
ydotd=0;

% Implementing PD gain to calculate desire roll and pitch angles
% Desire roll angle 
PDx=(kpx*(xd-x))+(kdx*(xdotd-xdot));
% Desire pitch angle without negative
PDy=(kpy*(yd-y))+(kdy*(ydotd-ydot));

% Desired altitude and acceleration in z-axis
zd=8;
zdotd=0;
% Desired roll angle and velocity 
thetad=PDx;
thetadotd=0;
% Desired pitch angle and velocity 
phid=-PDy;
phidotd=0;
% Desired yaw angle and velocity 
psid=0;
psidotd=0;

% Implementing PD gain to calculate desire roll and pitch angles
PDz=(kpz*(zd-z))+(kdz*(zdotd-zdot));
PDtheta=(kptheta*(thetad-theta))+(kdtheta*(thetadotd-thetadot));
PDphi=(kpphi*(phid-phi))+(kdphi*(phidotd-phidot));
PDpsi=(kppsi*(psid-psi))+(kdpsi*(psidotd-psidot));

% vehicle's forces and torques
Fz=PDz+(m*g);
tautheta=PDtheta;
tauphi=PDphi;
taupsi=PDpsi;

output = zeros(16, 1);

% To plot the current velocity and acceleration
% Velocity and acceleration in the x axis
output(1)=xdot;
output(2)=(Fz/m)*((theta*cos(psid))+(phi*sin(psid)));
% Velocity and acceleration in the y axis
output(3)=ydot;
output(4)=(Fz/m)*((theta*sin(psid))-(phi*cos(psid)));
% Velocity and acceleration in the z axis
output(5)=zdot;
output(6)=(Fz/m)-g;
% Angular velocity and acceleration for roll
output(7)=thetadot;
output(8)=tautheta/Jy;
% Angular velocity and acceleration for pitch
output(9)=phidot;
output(10)=tauphi/Jx;
% Angular velocity and acceleration for yaw
output(11)=psidot;
output(12)=taupsi/Jz;

% Updated Input force in z direction
% and torques acting on the quadcopter 
output(13)=Fz; 
output(14)=tautheta;
output(15)=tauphi;
output(16)=taupsi;
end