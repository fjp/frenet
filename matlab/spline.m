close all
clear all
clc
set(0, 'DefaultLineLineWidth', 1);
disp('Spline 2D test')
    
x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0];
y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0];


%x = [-2.5, 0.0, 2.5, 5.0, 7.5, 10.2, 15.3];
%y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0];
ds = 0.1;

[rx, ry, ryaw, rk, s] = CalcSplineCourse(x, y, ds);

figure
plot(x, y, 'xb', 'DisplayName', 'input');
hold on
plot(rx, ry, '-r', 'DisplayName', 'spline');
grid on;
%axis equal;
xlabel('x[m]');
ylabel('y[m]');
legend;

figure
plot(s, rad2deg(ryaw), '-r', 'DisplayName', 'yaw');
grid on;
legend;
xlabel('line length[m]');
ylabel('yaw angle[deg]');

figure
plot(s, rk, '-r', 'DisplayName', 'curvature');
grid on;
legend;
xlabel('line length[m]');
ylabel('curvature [1/m]');