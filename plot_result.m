
% plot
t = [1:N+1]*dt;

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

hull_gap = min([
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

wheel1_x_wrt_hull = wheel1x - xh;
wheel2_x_wrt_hull = wheel2x - xh;
wheel3_x_wrt_hull = wheel3x - xh;
force_wh1 = wheel_spring_force(wheel1_gap);
force_wh2 = wheel_spring_force(wheel2_gap);
force_wh3 = wheel_spring_force(wheel3_gap);
Fy = force_wh1 + force_wh2 + force_wh3;
Mz = -(wheel1_x_wrt_hull.*force_wh1 + wheel2_x_wrt_hull.*force_wh2 + wheel3_x_wrt_hull.*force_wh3);

figure(1)
clf()

subplot(531)
plot(t,xh)
legend('xh')
subplot(532)
plot(t,yh)
legend('yh')
subplot(533)
plot(t,thh/deg)
legend('thh')
subplot(534)
plot(t,vxh)
legend('vxh')
subplot(535)
plot(t,vyh)
legend('vyh')
subplot(536)
plot(t,dthh/deg)
legend('dthh')
subplot(537)
plot(t,tha3/deg)
legend('tha3')
subplot(538)
plot(t,tha2/deg)
legend('tha2')
subplot(539)
plot(t,tha1/deg)
legend('tha1')
subplot(5,3,10)
plot(t,dtha1/deg,t,dtha2/deg,t,dtha3/deg)
legend('dtha1','dtha2','dtha3')
subplot(5,3,11)
plot(t,wheel1_gap,t,wheel2_gap,t,wheel3_gap)
legend('wheel1_gap','wheel2_gap','wheel3_gap')
subplot(5,3,12)
plot(t,hull_gap)
legend('hull_gap')
subplot(5,3,14)
plot(t,Fy)
legend('Fy')
subplot(5,3,15)
plot(t,Mz)
legend('Mz')

