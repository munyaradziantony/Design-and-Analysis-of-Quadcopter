function output=Innerlooptest(input)

% Forces from the Outerloop control
Fz       = input(1);
tauy     = input(2);
taux     = input(3);
tauz     = input(4);

output = zeros(4, 1);

% Output velocity at each motor
output(1)= Fz-taux+tauy+tauz;
output(2)= Fz+taux-tauy+tauz;
output(3)= Fz-taux+tauy-tauz;
output(4)= Fz-taux-tauy-tauz;
end