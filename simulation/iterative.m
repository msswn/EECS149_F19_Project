clear all
close all

% Vehicle Parameters
l = 0.352;
Rmin = l/tan(50*pi/180);
d1 = l/2+0.01;

% Initialize waypoints
f1 = [0,0];
f2 = [0,0];
f4 = [0,0];
f5 = [0,0];
f6 = [0,0];
f7 = [0,0];
f8 = [0,0];
f9 = [0,0];

% Parking spot parameters
spotLength = l+2.6*d1;
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
yi = 0.5;
plot(xi,yi,'.','MarkerSize',20)
circleFull(xi,yi,l/2)

% Final position
xf = spotBack;
yf = 0;

% Initial position offset
S = xi-xf;
H = yi-yf;

% Calculate segment and tangent point 1
k = (S*(H-2*Rmin)+sqrt(4*Rmin^2*(S^2+H^2)-16*Rmin^3*H))/(S^2-4*Rmin^2);
m = Rmin*(1-sqrt(1+k^2))+yf;
f1 = [S-k/sqrt(1+k^2)*Rmin, H-(1-1/sqrt(1+k^2))*Rmin];
f1 = f1 + [xf,yf];

% Plot C1
circle(S+xf,H-Rmin+yf,Rmin,S+xf,H+yf,f1(1),f1(2));
plot(f1(1),f1(2),'.','MarkerSize',20)

% Calculate tangent point 2
f2 = [k/sqrt(1+k^2)*Rmin, (1-1/sqrt(1+k^2))*Rmin];
f2 = f2 + [xf,yf];

% Check if S exceeds bounds
if f2(1) < spotBack + d1
    % Vehicle stops before collision
    f2 = [spotBack + d1, k*(spotBack+d1-xf) + m + yf];
    plot(f2(1),f2(2),'.','MarkerSize',20)
    circleFull(f2(1),f2(2),l/2)
    f = [f1; f2];
    
    % Calculate and plot segment
    xS = linspace(f1(1),f2(1));
    yS = k*(xS-xf) + m + yf;
    plot(xS,yS,'k')
    
    % Calculate and plot first correction turn
    dS = f1-f2;
    dSn = [dS(2) -dS(1)]/norm(dS);
    cx = Rmin*dSn(1) + f2(1);
    cy = Rmin*dSn(2) + f2(2);
    f5 = [cx cy] + [-(f2(1)-cx) f2(2)-cy];
    circle(cx, cy, Rmin,f2(1),f2(2),f5(1),f5(2));
else
    f = [f1; f2];
    
    xS = linspace(f1(1),f2(1));
    yS = k*(xS-xf) + m + yf;
    
    plot(f(:,1),f(:,2),'.k')
    plot(xS,yS,'k')
    plot(xi,yi,'k')
    
    axis equal
    
    newO = [spotBack+d1 0];
    
    % Vehicle stops before collision
    f4 = [0, Rmin-sqrt(Rmin^2-d1^2)];
    f4 = f4+newO;
    plot(f4(1),f4(2),'.','MarkerSize',20)
    circleFull(f4(1),f4(2),l/2)
    circle(xf,Rmin+yf,Rmin,f4(1),f4(2),f2(1),f2(2));
    
    % Calculate and plot first correction turn
    dS = [f4(1)-xf f4(2)-(Rmin+yf)];
    dSn = [dS(1) dS(2)]/norm(dS);
    cx = Rmin*dSn(1) + f4(1);
    cy = Rmin*dSn(2) + f4(2);
    f5 = [cx + (cx-f4(1)), f4(2)];
    circle(cx, cy,Rmin,f4(1),f4(2),f5(1),f5(2));
end

plot(f5(1),f5(2),'.','MarkerSize',20)

% Calculate second correction turn
f6xd = f5(1) + sqrt(Rmin^2-(Rmin-f5(2))^2);
f6y = 0;
f6 = [f6xd f6y];
% plot(f6(1),f6(2),'.','MarkerSize',20)
% circle(f6xd, Rmin, Rmin,f5(1),f5(2),f6(1),f6(2));

% Check for collision during second correction turn
if f6(1) >= spotFront - d1
    f6x = spotFront - d1;
    f6y = Rmin - sqrt(Rmin^2-(f6x-f6xd)^2);
    f6 = [f6x, f6y];
    cx = f6xd;
    cy = Rmin;
    plot(f6(1),f6(2),'.','MarkerSize',20)
    circleFull(f6(1),f6(2),l/2);
    circle(cx, cy, Rmin,f5(1),f5(2),f6(1),f6(2));
    cx = f6(1)-(f6xd-f6(1));
    cy = f6(2)-(Rmin-f6(2));
    f7 = [cx cy] + [-(f6(1)-cx) f6(2)-cy];
    f8 = [f7(1)-sqrt(Rmin^2-(Rmin-f7(2))^2) 0];
    plot(f7(1),f7(2),'.','MarkerSize',20)
    circle(cx, cy, Rmin,f6(1),f6(2),f7(1),f7(2));
    plot(f8(1),f8(2),'.','MarkerSize',20)
    circle(f8(1), Rmin, Rmin,f7(1),f7(2),f8(1),f8(2));
    circleFull(f8(1),f8(2),l/2);
    f9 = spotCenter;
    plot([f8(1) f9(1)], [f8(2) f9(2)],'k')
    plot(f9(1),f9(2),'.','MarkerSize',20)
    circleFull(f9(1),f9(2),l/2);
else
    cx = f6xd;
    cy = Rmin;
    plot(f6(1),f6(2),'.','MarkerSize',20)
    plot(f6(1),f6(2),'.','MarkerSize',20)
    circle(cx, cy, Rmin,f5(1),f5(2),f6(1),f6(2));
    circleFull(f6(1),f6(2),l/2);
    f9 = spotCenter;
    plot([f6(1) f9(1)], [f6(2) f9(2)],'k')
    plot(f9(1),f9(2),'.','MarkerSize',20)
    circleFull(f9(1),f9(2),l/2);
end

title(['x_i = ',num2str(xi),', y_i = ',num2str(yi)])
% set(gca,'FontName','SF Pro Text','FontSize',5)

[f1; f2; f4; f5; f6; f7; f8; f9]
