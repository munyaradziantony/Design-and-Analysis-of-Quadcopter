function output=Innerloop(input)

% Outerloop first channel
z          = input(1);
zd         = input(2);
xdot       = input(3);
ydot       = input(4);
zdot       = input(5);
zdotd      = input(6);
phid       = input(7);
phidotd    = input(8);
thetad     = input(9);
thetadotd  = input(10);
psid       = input(11);
psidotd    = input(12);
sim_t      = input(13);
% Feedback second channel
phi       = input(14);
phidot    = input(15);
theta     = input(16);
thetadot  = input(17);
psi       = input(18);
psidot    = input(19);

% Innerloop PD controller gain
% Control gains for z
kpz=3;   kdz=3.5;
% Control gains for roll
kptheta=5 ; kdtheta=2;
% Control gains for pitch
kpphi=5;   kdphi=2;
% Control gains for yaw
kppsi=5;   kdpsi=0;

% Constant parameters: mass, gravity, inertial values
m=1;
g=9.81;
Jx=0.2;
Jy=0.2;
Jz=0.2;

% Implementing PD gain to calculate desire roll and pitch angles
PDz=(kpz*(zd-z))+(kdz*(zdotd-zdot));
PDtheta=(kptheta*(thetad-theta))+(kdtheta*(thetadotd-thetadot));
PDphi=(kpphi*(phid-phi))+(kdphi*(phidotd-phidot));
PDpsi=(kppsi*(psid-psi))+(kdpsi*(psidotd-psidot));

% vehicle's forces and torques
Fz=PDz+(m*g);
tauphi=PDphi;
tautheta=PDtheta;
taupsi=PDpsi;

%Initializing the outputs
output = zeros(16, 1);

% For the computing drone dynamic(Motor speed)
output(1)=Fz;
output(2)=tauphi;
output(3)=tautheta;
output(4)=taupsi;
output(5)=sim_t;

% Feedback to Innerloop
% Angular velocity and acceleration for pitch
output(6)=phidot;
output(7)=tauphi/Jx;
% Angular velocity and acceleration for roll
output(8)=thetadot;
output(9)=tautheta/Jy;
% Angular velocity and acceleration for yaw
output(10)=psidot;
output(11)=taupsi/Jz;

% Feedback to Outerloop
% Velocity in x,y and z axis
output(12)=xdot;
output(13)=ydot;
output(14)=zdot;
% Acceleration in x,y and z axis 
output(15)=(Fz/m)*((theta*cos(psid))+(phi*sin(psid)));
output(16)=(Fz/m)*((theta*sin(psid))-(phi*cos(psid)));
output(17)=(Fz/m)-g;

end