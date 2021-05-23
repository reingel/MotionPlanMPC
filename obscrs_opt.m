clear all

global nx nu N dt;
global Lh Hh Ls Hs La Rw Wr;

g = 9.80665;
deg = pi/180;
cm = 1/100;

Lh = 20*cm; % hull length
Hh = 6*cm; % hull height
Ls = [16*cm, 0*cm, -16*cm]; % shoulder gap x from hull CG
Hs =  4*cm; % shoulder gap z from hull CG
La =  8*cm; % arm length
Rw =  6*cm; % wheel radius
mh = 1.5;
mw = 0.2;
Ih = 1;
Iw = 0.2;
Wr = mh + 3*mw;

nx = 6;
nu = 6;
n = nx + nu;

N = 40;
dt = 0.05;

road_amp = 0.01;
road_freq = 1;

mh = 1.5;
mw = 0.2;
mr = mh + 3*mw;
La = 0.08;
Ia = mw*La^2;

% x = zeros(n, N+1);

fun = @(x) -x(1,end) + ...
    sqrt(mean((x(1,2:end) - x(1,1:end-1)).^2)) + ...
    sqrt(mean((x(2,2:end) - x(2,1:end-1)).^2)) + ...
    sqrt(mean((x(3,2:end) - x(3,1:end-1)).^2)) + ...
    sqrt(mean((x(4,2:end) - x(4,1:end-1)).^2)) + ...
    sqrt(mean((x(5,2:end) - x(5,1:end-1)).^2)) + ...
    sqrt(mean((x(6,2:end) - x(6,1:end-1)).^2));

x0 = zeros(n, N+1);
x0(2,:) = 0.14;
x0(4,:) = -60*deg;
x0(5,:) = 60*deg;
x0(6,:) = 60*deg;

lb = -Inf*ones(n, N+1);
ub = Inf*ones(n, N+1);

lb(1,:) = -0.5;
lb(2,:) = 0;
lb(3,:) = -45*deg;
lb(4:5,:) = -90*deg;
lb(6,:) = 0*deg;
lb(1+6,:) = 0;
lb(2+6,:) = -0.5;
lb((3:6)+6,:) = -10*deg;

ub(1,:) = 3;
ub(2,:) = 10;
ub(3,:) = 45*deg;
ub(4,:) = 0*deg;
ub(5:6,:) = 90*deg;
ub(1+6,:) = 1;
ub(2+6,:) = 0.5;
ub((3:6)+6,:) = 10*deg;


options = optimoptions('fmincon');
options.MaxFunctionEvaluations = 100000;
options.MaxIterations = 1e3;
options.ConstraintTolerance = 1e-6;

[x, fval] = fmincon(fun, x0, [], [], [], [], lb, ub, @nonlcon, options);
disp(fval);

plot_result;
disp('plot');

