function output = quatcompute(input)
    phi     = input(1)/2;     % Pitch angle in radians
    theta   = input(2)/2;     % Roll angle in radians
    psi     = input(3)/2;     % Yaw angle in radians

    % Calculate quaternion components
    q0 = cos(phi) * cos(theta) * cos(psi) + sin(phi) * sin(theta) * sin(psi);
    q1 = sin(phi) * cos(theta) * cos(psi) - cos(phi) * sin(theta) * sin(psi);
    q2 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * cos(theta) * sin(psi);
    q3 = cos(phi) * cos(theta) * sin(psi) - sin(phi) * sin(theta) * cos(psi);

    % Initializing the output vector
    output = [q0; q1; q2; q3];
end
