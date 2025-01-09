function output=LQR(input)

% Current states
phie       = input(1);
phidote    = input(2);
thetae     = input(3);
thetadote  = input(4);
psie       = input(5);
psidote    = input(6);
xe         = input(7);
ye         = input(8);
ze         = input(9);
xdote      = input(10);
ydote      = input(11);
zdote      = input(12);
sim_t     = input(13);

% Desired states
xd         = input(14);
yd         = input(15);
zd         = input(16);
xdotd      = input(17);
ydotd      = input(18);
zdotd      = input(19);
phid       = input(20);
phidotd    = input(21);
thetad     = input(22);
thetadotd  = input(23);
psid       = input(24);
psidotd    = input(25);

% Current Force and Torques
Fz         = input(26);
tauphi     = input(27);
tautheta   = input(28);
taupsi     = input(29);

current_estate = [xe ye ze xdote ydote zdote phie phidote thetae thetadote psie psidote];
desired_state = [xd yd zd xdotd ydotd zdotd phid phidotd thetad thetadotd psid psidotd];
current_u = [Fz; tauphi; tautheta; taupsi];

% Constant parameters: mass, gravity, inertial values
m=1;
g=9.81;
Jx=0.2;
Jy=0.2;
Jz=0.2;

% Define system matrices A and B
A = [0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 -g 0 0 0 0;
     0 0 0 0 0 0 g 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;];

B = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     1/m 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 1/Jx 0 0;
     0 0 1/Jy 0;
     0 0 0 1/Jx];

Q = diag([1 1 1 10 10 10 1 1 1 10 10 10]);
R = diag([1 1 1 1]);

% Solving the algebraic Riccati equation
[X,L,G] = dare(A, B, Q, R);

% calculating estimated forces 
u = G * (desired_state - current_estate)' + current_u;

%Initializing the outputs
output = zeros(4, 1);
output(1) = u(1);
output(2) = u(2);
output(3) = u(3);
output(4) = u(4);
end