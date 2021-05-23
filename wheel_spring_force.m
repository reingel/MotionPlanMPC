function fs = wheel_spring_force(distance)
fN0 = 0.1;
fN_hat = 1000;
gN_hat = 0.01;
fs = fN0 * (fN_hat / fN0).^(-distance / gN_hat);
