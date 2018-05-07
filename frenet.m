close all
clear all
clc
set(0, 'DefaultLineLineWidth', 2);

nPoints = 20;

x = zeros(1,nPoints);
y = zeros(1,nPoints);

xRef = zeros(1,nPoints);
yRef = zeros(1,nPoints);
psiRef = zeros(1,nPoints);
kappaRef = zeros(1,nPoints);
lCumRef = zeros(1,nPoints);

deltaX = zeros(1,nPoints);
deltaY = zeros(1,nPoints);

%% Generate reference path
dx = 2;
dy = 1;
m = 0.5;

xRef(1) = -2;
yRef(1) = dy;
for i = 2:nPoints
    xRef(i) = xRef(i-1) + dx;
    yRef(i-1) = m*xRef(i-1) + dy; 
end
yRef(end) = m*xRef(end) + dy;

for i = 1:nPoints
    iPrev = i - 1;
    if (iPrev < 1)
        iPrev = 0;
    end
    iNext = i + 1;
    if (iNext > nPoints)
        iNext = nPoints;
    end
    deltaX(i) = xRef(iNext) - xRef(i);
    deltaY(i) = yRef(iNext) - yRef(i);
    psi = atan2(deltaY(i), deltaX(i));
    psiRef(i) = psi;
    
    % Todo use wiki formula
    kappaRef = zeros(1,nPoints);
end
psiRef(end) = psiRef(end-1);
%psiRef(1) = 0;

lCum = sqrt(deltaX(1)^2 + deltaY(1)^2);
nSign = (xRef(1) >= 0) - (xRef(1) < 0);
lCumRef(1) = lCum*nSign;
for i = 2:nPoints
    lCum = sqrt(deltaX(i)^2 + deltaY(i)^2);
    nSign = (xRef(i) >= 0) - (xRef(i) < 0);
    lCumRef(i) = lCumRef(i-1) + lCum*nSign;
end

figure
hr = plot(xRef, yRef, 'Color', [0.1, 0.1, 0.1], 'Marker', 'o', 'DisplayName', 'Ref');
grid on;
hold on;

% Plot line from origin orthogonal to to reference path (intersection)
L1 = [xRef; yRef];
L2 = [-xRef; -1/m*(-xRef)];
P = intersect(L1, L2);
xMin = P(1);
yMin = P(2);
%plot([0, xRef(1), 0, xRef(2), 0, xRef(3)], [0, yRef(1), 0, yRef(2), 0, yRef(3)]);
plot([0, xMin], [0, yMin]);

shrink = .3;
axis([min(xRef)-1, max(xRef)*shrink, min(yRef)-1, max(yRef)*shrink]);
axis equal;

% end time
T = 1; % seconds

%% s coordinate
xx = abs(xMin-xRef(1));
yy = abs(yMin-yRef(1));
quiver(xRef(1), yRef(1), xx, yy);
s0 =  lCumRef(1) + sqrt(xx^2 + yy^2);
%s0 = 0;
ds0 = 15;
dds0 = 0;

sT = 10;
dsT = 10;
ddsT = 0;

% s coefficents
cs0 = s0;
cs1 = ds0;
cs2 = dds0/2;

M1 = [1, T, T*T; 
      0, 1, 2*T; 
      0, 0, 2  ];
  
M2 = [  T^3,    T^4,   T^5;
      3*T^2,  4*T^3, 5*T^4;
        6*T, 12*T^2, 20*T^3];

cs = M2\([sT; dsT; ddsT] - M1 * [cs0; cs1; cs2]);
cs3 = cs(1);
cs4 = cs(2);
cs5 = cs(3);

t = linspace(0,T,20);
s = cs0 + cs1.*t + cs2.*t.^2 + cs3.*t.^3 + cs4.*t.^4 + cs5.*t.^5;

%% d coordinate
d0 = -sqrt(xMin.^2 + yMin.^2);%-dy;
dd0 = 0;
ddd0 = 0;

dT = 0;
ddT = 0;
dddT = 0;

% d coefficients
cd0 = d0;
cd1 = dd0;
cd2 = ddd0/2;

cd = M2\([dT; ddT; dddT] - M1 * [cd0; cd1; cd2]);
cd3 = cd(1);
cd4 = cd(2);
cd5 = cd(3);


d = cd0 + cd1.*t + cd2.*t.^2 + cd3.*t.^3 + cd4.*t.^4 + cd5.*t.^5;


hf = plot(s,d, 'Color', 'g', 'Marker', 'o', 'DisplayName', 'Frenet (s,d)');


%% Transform Frenet (s,d) to Cartesian (local) vehicle frame (x,y)

for i = 1:nPoints
    ps = s(i);
    pd = d(i);
    
    idx = 1;
    
    while(ps > lCumRef(idx) && (idx < nPoints - 1))
        idx = idx + 1;
    end
    
    idxNext = idx + 1;
    
    ds = ps - lCumRef(idx);
    
    psi = psiRef(idx);
    
    segX = xRef(idx) + ds * cos(psi);
    segY = yRef(idx) + ds * sin(psi);
    
    
    
    x(i) = segX - pd * sin(psi);
    y(i) = segY + pd * cos(psi);
end


hc = plot(x, y, 'Color', 'b', 'Marker', 'o', 'DisplayName', 'Cart (x,y)');

legend([hr, hf, hc], 'Location', 'best');