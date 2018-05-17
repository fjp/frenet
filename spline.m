close all
clear all
clc
set(0, 'DefaultLineLineWidth', 1);
disp('Spline 2D test')
    
x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0];
y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0];


x = [-2.5, 0.0, 2.5, 5.0, 7.5, 10.2, 15.3];
y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0];
ds = 0.1;

[rx, ry, ryaw, rk, s] = calc_spline_course(x, y, ds);

figure
plot(x, y, 'xb', 'DisplayName', 'input');
hold on
plot(rx, ry, '-r', 'DisplayName', 'spline');
grid on;
axis equal;
xlabel('x[m]');
ylabel('y[m]');
legend;

figure
plot(s, rad2deg(ryaw), '-r', 'DisplayName', 'yaw');
grid on;
axis equal;
legend;
xlabel('line length[m]');
ylabel('yaw angle[deg]');

figure
plot(s, rk, '-r', 'DisplayName', 'curvature');
grid on;
legend;
xlabel('line length[m]');
ylabel('curvature [1/m]');


function [rx, ry, ryaw, rk, s] = calc_spline_course(x, y, ds) % d=0.1 default
    sp = cSpline2D(x, y);
    s = 0:ds:sp.s(end);

    rx = [];
    ry = [];
    ryaw = [];
    rk = [];
    for i_s = s
        [ix, iy] = sp.calc_position(i_s);
        rx(end+1) = ix;
        ry(end+1) = iy;
        ryaw(end+1) = sp.calc_yaw(i_s);
        rk(end+1) = sp.calc_curvature(i_s);
    end
end