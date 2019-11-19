clear all
close all

l = 0.352;
Rmin = l/tan(50*pi/180);
d1 = l/2;

spotLength = l*1.5;
spotWidth = l*1.25;
spotCenter = [0 0];
spotBack = spotCenter(1)-spotLength/2;
spotFront = spotCenter(1)+spotLength/2;
spotLeft = spotCenter(2)+spotWidth/2;
spotRight = spotCenter(2)-spotWidth/2;
rect(spotLength,spotWidth,spotCenter)
hold on

xf = -0.2;
yf = 0;

xi = 1;
yi = 0.75;

S = xi-xf;
H = yi-yf;

k = (S*(H-2*Rmin)+sqrt(4*Rmin^2*(S^2+H^2)-16*Rmin^3*H))/(S^2-4*Rmin^2);
m = Rmin*(1-sqrt(1+k^2))+yf;
f1 = [S-k/sqrt(1+k^2)*Rmin, H-(1-1/sqrt(1+k^2))*Rmin];
f1 = f1 + [xf,yf];
f2 = [k/sqrt(1+k^2)*Rmin, (1-1/sqrt(1+k^2))*Rmin];
f2 = f2 + [xf,yf];
f = [f1; f2];

d1 = l/2;

xS = linspace(f1(1),f2(1));
yS = k*(xS-xf) + m + yf;

circle(S+xf,H-Rmin+yf,Rmin,S+xf,H+yf,f1(1),f1(2));
hold on
circle(xf,Rmin+yf,Rmin,xf,yf,f2(1),f2(2));
hold on

plot(0,0,'.r','MarkerSize',12)
plot(xf,yf,'.r','MarkerSize',12)
plot(f(:,1),f(:,2),'.k')
plot(xS,yS,'k')
plot(xi,yi,'k')
axis equal

f4 = [0, Rmin-sqrt(Rmin^2-d1^2)];
f5 = [2*d1, Rmin-sqrt(Rmin^2-d1^2)];
f6 = [3*d1,0];

f4 = f4+[d1+spotBack 0];
circleFull(f4(1),f4(2),l/2);
f5 = f5 + [d1+spotBack 0];
f6 = f6+[d1+spotBack 0];

plot(f4(1),f4(2),'.','MarkerSize',20)
plot(f5(1),f5(2),'.','MarkerSize',20)
plot(f6(1),f6(2),'.','MarkerSize',20)

axis equal