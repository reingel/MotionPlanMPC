function [c, ceq] = nonlcon(x)
global nx nu N dt;
global Lh Hh Ls Hs La Rw Wr;

xh = x(1,:);
yh = x(2,:);
thh = x(3,:);
tha1 = x(4,:);
tha2 = x(5,:);
tha3 = x(6,:);
vxh = x(1+nx,:);
vyh = x(2+nx,:);
dthh = x(3+nx,:);
dtha1 = x(4+nx,:);
dtha2 = x(5+nx,:);
dtha3 = x(6+nx,:);

c   = zeros(  (N+1)+0*N, 1);
ceq = zeros(2*(N+1)+6*N, 1);

% --- inequality constraints ---
% hull contact constraints
c(1:N+1) = road_height(xh) - min([
    yh + Lh*sin(-thh) + Hh*cos(-thh)
    yh - Lh*sin(-thh) + Hh*cos(-thh)
    yh + Lh*sin(-thh) - Hh*cos(-thh)
    yh - Lh*sin(-thh) - Hh*cos(-thh)
    ]);

wheel1x = xh + Ls(1)*cos(-thh) - Hs*sin(-thh) + La*sin(-thh-tha1);
wheel2x = xh + Ls(2)*cos(-thh) - Hs*sin(-thh) + La*sin(-thh-tha2);
wheel3x = xh + Ls(3)*cos(-thh) - Hs*sin(-thh) + La*sin(-thh-tha3);
wheel1y = yh + Ls(1)*sin(-thh) - Hs*cos(-thh) - La*cos(-thh-tha1);
wheel2y = yh + Ls(2)*sin(-thh) - Hs*cos(-thh) - La*cos(-thh-tha2);
wheel3y = yh + Ls(3)*sin(-thh) - Hs*cos(-thh) - La*cos(-thh-tha3);

wheel1_gap = wheel1y - Rw - road_height(xh);
wheel2_gap = wheel2y - Rw - road_height(xh);
wheel3_gap = wheel3y - Rw - road_height(xh);

% static equilibrium constraints
wheel1_x_wrt_hull = wheel1x - xh;
wheel2_x_wrt_hull = wheel2x - xh;
wheel3_x_wrt_hull = wheel3x - xh;
force_wh1 = wheel_spring_force(wheel1_gap);
force_wh2 = wheel_spring_force(wheel2_gap);
force_wh3 = wheel_spring_force(wheel3_gap);
Fy = force_wh1 + force_wh2 + force_wh3;
Mz = -(wheel1_x_wrt_hull.*force_wh1 + wheel2_x_wrt_hull.*force_wh2 + wheel3_x_wrt_hull.*force_wh3);
ceq(1:N+1) = Fy - Wr;
ceq((1:N+1)+(N+1)) = Mz;

for i = 1:N
    % --- inequality constraints ---
%     c(i+3*(N+1)) = (vxh(i+1) - vxh(i))^2 - 1;
%     c(i+3*(N+1)+N) = (vyh(i+1) - vyh(i))^2 - 1;
%     c(i+3*(N+1)+2*N) = (dthh(i+1) - dthh(i))^2 - 1;
%     c(i+3*(N+1)+3*N) = (dtha1(i+1) - dtha1(i))^2 - 1;
%     c(i+3*(N+1)+4*N) = (dtha2(i+1) - tha2(i))^2 - 1;
%     c(i+3*(N+1)+5*N) = (dtha3(i+1) - dtha3(i))^2 - 1;

    % --- equality constraints ---
    % dynamics
    % X(k+1) = X(k) + U(k)*dt
    ceq(i+2*(N+1)    ) = x(1,i+1) - x(1,i) - x(1+nx,i)*dt;
    ceq(i+2*(N+1)+  N) = x(2,i+1) - x(2,i) - x(2+nx,i)*dt;
    ceq(i+2*(N+1)+2*N) = x(3,i+1) - x(3,i) - x(3+nx,i)*dt;
    ceq(i+2*(N+1)+3*N) = x(4,i+1) - x(4,i) - x(4+nx,i)*dt;
    ceq(i+2*(N+1)+4*N) = x(5,i+1) - x(5,i) - x(5+nx,i)*dt;
    ceq(i+2*(N+1)+5*N) = x(6,i+1) - x(6,i) - x(6+nx,i)*dt;
end
