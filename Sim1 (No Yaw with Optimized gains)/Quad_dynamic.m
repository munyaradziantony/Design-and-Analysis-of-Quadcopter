function output=Quad_dynamic(input)

% Forces from the Outerloop control
Fz       = input(1);
taux     = input(2);
tauy     = input(3);
tauz     = input(4);
sim_t    = input(5);

output = zeros(4, 1);

W=[1,-1,1,1;1,1,-1,1;1,1,1,-1;1,-1,-1,-1]*[Fz;tauy;taux;tauz];

% Output velocity at each motor
output(1)= W(1);
output(2)= W(2);
output(3)= W(3);
output(4)= W(4);
output(5)= sim_t;
end