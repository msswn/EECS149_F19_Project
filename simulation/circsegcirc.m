clear all
close all

% Vehicle Parameters
l = 0.352;
Rmin = l/tan(50*pi/180);
d1 = l/2+0.01;

% Parking spot parameters
spotLength = l+3*d1;
spotWidth = l*1.25;
spotCenter = [0 0];
spotBack = spotCenter(1)-spotLength/2;
spotFront = spotCenter(1)+spotLength/2;
spotLeft = spotCenter(2)+spotWidth/2;
spotRight = spotCenter(2)-spotWidth/2;
rect(spotLength,spotWidth,spotCenter);
hold on

% Initial position
xi = 0.5;
yi = 0.6;
plot(xi,yi,'.','MarkerSize',20)
circleFull(xi,yi,l/2)

% Final position
xf = spotBack + d1;
yf = 0;
circleFull(xf,yf,l/2);

% Initial position offset
S = xi-xf;
H = yi-yf;

% Calculate segment and tangent point 1
k = (S*(H-2*Rmin)+sqrt(4*Rmin^2*(S^2+H^2)-16*Rmin^3*H))/(S^2-4*Rmin^2);
m = Rmin*(1-sqrt(1+k^2))+yf;
f1 = [S-k/sqrt(1+k^2)*Rmin, H-(1-1/sqrt(1+k^2))*Rmin];
f1 = f1 + [xf,yf];
circleFull(f1(1),f1(2),l/2);

% Plot C1
circle(S+xf,H-Rmin+yf,Rmin,S+xf,H+yf,f1(1),f1(2));
plot(f1(1),f1(2),'.','MarkerSize',20)
th1 = th(S,H-Rmin, Rmin, S, H, f1(1), f1(2));
magC1 = Rmin * th1;

% Calculate tangent point 2
f2 = [k/sqrt(1+k^2)*Rmin, (1-1/sqrt(1+k^2))*Rmin];
f2 = f2 + [xf,yf];
% th2 = th(xf,yf+Rmin,Rmin,f2(1),f2(2),xf,yf);
th2 = th1;
magC2 = Rmin * th2;
plot(f2(1),f2(2),'.','MarkerSize',20)
circleFull(f2(1),f2(2),l/2);
circle(xf,yf+Rmin,Rmin,f2(1),f2(2),xf,yf)
plot(xf,yf,'.','MarkerSize',20)

f = [f1; f2];

xS = linspace(f1(1),f2(1));
yS = k*(xS-xf) + m + yf;
magS = norm(f1-f2);

plot(f(:,1),f(:,2),'.k')
plot(xS,yS,'k')
plot(xi,yi,'k')

axis equal

newO = [spotBack+d1 0];

vel = 100;
t1 = magC1*1000/vel;
t2 = magS*1000/vel;
t3 = magC2*1000/vel;

t1
t2
t3