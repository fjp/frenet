close all
clear all
clc
set(0, 'DefaultLineLineWidth', 1);
disp('Optimal Frenet Path Planning')

% way points
WPx = [00.0, 20.0, 30.0, 50.0];
WPy = [00.0, 00.75, 02.15, 03.0];

WPx = [00.0, 20.0, 30.0, 50.0];
WPy = [00.0, 00.5, 04.5, 07.0];

%% Define obstacles in objects lists
% objx = objects(:,1);
% objy = objects(:,2);
faObjects = [10.0, -0.5;
    15.0, 2.0;
    20.0, 0.0;
    30.0, 3.0;
    45.0, 3.5];

%% Create a reference path
ds = 0.1;
GenerateTargetCourse = @(wx, wy) CalcSplineCourse(wx, wy, ds);
[faRefX, faRefY, faRefYaw, faRefCurvature, faRefRunLength, oReferencePath] = ...
    GenerateTargetCourse(WPx, WPy);


figure;
subplot(2,1,1);

plot(WPx, WPy, '.');
hold on
faColorRef = [0.01, 0.1, 0.9];
hru = plot(faRefX, faRefY, '-', 'color', faColorRef, 'DisplayName', 'Reference');
title('USK', 'Interpreter', 'latex');
xlabel('x', 'Interpreter', 'latex');
ylabel('y', 'Interpreter', 'latex');

hou = plot(faObjects(:,1), faObjects(:,2), 'kx', 'DisplayName', 'Objects');
axis equal;


hObj = @(ox,oy) [ox-1, oy+1;
        ox-1, oy+0.5;
        ox-1, oy;
        ox-1, oy-0.5;
        ox-1, oy-1;
        ox, oy-1;
        ox+1, oy-1;
        ox+2, oy-1;
        ox+3, oy-1;
        ox+3, oy-0.5;
        ox+3, oy;
        ox+3, oy+0.5;
        ox+3, oy+1;
        ox+2, oy+1;
        ox+1, oy+1;
        ox, oy+1;
        ox-1, oy+1];

for i = 1:length(faObjects)

    fObjX = faObjects(i,1);
    fObjY = faObjects(i,2);
    
    plot(fObjX, fObjY, 'kx');
    
    
    faObj = hObj(fObjX, fObjY);


    fObjX = faObj(:,1);
    fObjY = faObj(:,2);
    
    plot(fObjX, fObjY, 'k.-');

end

hl = legend([hru, hou], 'Location', 'best');
set(hl, 'Interpreter', 'latex');

subplot(2,1,2);

hrf = plot([0, faRefRunLength(end)], [0, 0], '-', 'color', faColorRef, 'DisplayName', 'Reference');
title('FRT', 'Interpreter', 'latex');
xlabel('s', 'Interpreter', 'latex');
ylabel('d', 'Interpreter', 'latex');
axis equal
hold on;
for i = 1:length(faObjects)
    fObjX = faObjects(i,1);
    fObjY = faObjects(i,2);
    
    [fObjS, fObjD] = Cart2FRT(fObjX, fObjY, 0, faRefX, faRefY);
    
    hof = plot(fObjS, fObjD, 'kx', 'DisplayName', 'Objects');
    
    
    faObj = hObj(fObjX, fObjY);

    nObjPoints = size(faObj,1);
    faObjS = zeros(1, nObjPoints);
    faObjD = zeros(1, nObjPoints);
    for j = 1:nObjPoints
        fObjX = faObj(j,1);
        fObjY = faObj(j,2);
    
        [fObjS, fObjD] = Cart2FRT(fObjX, fObjY, 0, faRefX, faRefY);
        faObjS(j) = fObjS;
        faObjD(j) = fObjD;
        
    end
    
    plot(faObjS, faObjD, 'k.-');
    
end

hl = legend([hrf, hof], 'Location', 'best');
set(hl, 'Interpreter', 'latex');

fObjX = faObjects(1,1)
fObjY = faObjects(1,2)

[fObjS, fObjD] = Cart2FRT(fObjX, fObjY, 0, faRefX, faRefY);

[fX, fY] = FRT2Cart(fObjS, fObjD, faRefRunLength, faRefX, faRefY)



fObjX == fX
fObjY == fY
