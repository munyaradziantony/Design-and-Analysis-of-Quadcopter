function output=Outerloop(input)

% Sorting the Inputs of 1st channel
xd      =   input(1); % used
yd      =   input(2); % used
zd      =   input(3);
psid    =   input(4);
psidotd =   input(6);
sim_t   =   input(6);

% Sorting actual position and velocity 
x      =   input(7);    % used
y      =   input(8);    % used
z      =   input(9);
xdot   =   input(10);   % used
ydot   =   input(11);   % used
zdot   =   input(12);

% Desired velocity in x and y
xdotd   =   0;
ydotd   =   0;
zdotd   =   0;

% Control gains for x
kpx=0.000625; kdx=0.0005;
% Control gains for y
kpy=0.000625; kdy=0.0005;

% Implementing PD controller
% x-direction
PDx=(kpx*(xd-x))+(kdx*(xdotd-xdot));
% y-direction
PDy=(kpy*(yd-y))+(kdy*(ydotd-ydot));

% Desired roll angle and speed: 
thetad=PDx;
thetadotd=0;
% Desired pitch angle 
phid=-PDy;
phidotd=0;

%Initializing the outputs
output = zeros(11, 1);

output(1)=z;
output(2)=zd;
output(3)=xdot;
output(4)=ydot;
output(5)=zdot;
output(6)=zdotd;
output(7)=phid;
output(8)=phidotd;
output(9)=thetad;
output(10)=thetadotd;
output(11)=psid;
output(12)=psidotd;
output(13)=sim_t;
end