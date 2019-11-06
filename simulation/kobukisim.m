close all
clear all

% All units in mm

% radius
l = 352;
% Track width
trackwidth = 230;

v = 50;
tEnd = 10;
p0 = [0 0];

t = 0:0.1:tEnd;
deltat = t(2)-t(1);
numTimeSteps = length(t);

thetas = zeros(numTimeSteps,1);
steeringAngle = zeros(numTimeSteps,1);
steeringAngle(1:end) = 30*(pi/180);
% steeringAngle(66:end) = -30*(pi/180);

positions = zeros(numTimeSteps,2);

x0 = 1;
y0 = 1;
theta0 = 0;
theta0 = theta0*pi/180;

positions(1,:) = [x0 y0];
thetas(1) = theta0;

for i = 2:numTimeSteps
    phi = steeringAngle(i);
    xdot = v*cos(thetas(i-1));
    ydot = v*sin(thetas(i-1));
    thetadot = 1/l*tan(phi)*v;
    xp = positions(i-1,1);
    yp = positions(i-1,2);
    thetap = thetas(i-1);
    x = xp + xdot*deltat;
    y = yp + ydot*deltat;
    theta = thetap + thetadot*deltat;
    positions(i,:) = [x y];
    thetas(i,:) = theta;
end

for i = 2:numTimeSteps
    c = positions(i,:);
    phi = steeringAngle(i);
    
    Rtheta = [cos(thetas(i)) -sin(thetas(i)); sin(thetas(i)) cos(thetas(i))];
    Rphi = [cos(thetas(i)+phi) -sin(thetas(i)+phi); sin(thetas(i)+phi) cos(thetas(i)+phi)];
%     
%     d = (Rtheta*[0.5; 0])';
    
    % car body boundaries
    figure(1)
    plot(c(1),c(2),'or','MarkerSize',100)
    hold on
    plot(c(1),c(2),'.r','MarkerSize',20)
    hold on
    
    % tires
    right = c + (Rtheta*[0; -trackwidth/2])';
    left = c + (Rtheta*[0; trackwidth/2])';
    tirecoords = [right; left];
    b = (Rtheta*[0.4; 0])';
    
    rightrear = right-b;
    rightfront = right+b;
    leftrear = left-b;
    leftfront = left+b;
    
    plot([rightrear(1) rightfront(1)],[rightrear(2) rightfront(2)],'k','LineWidth',2)
    plot([leftrear(1) leftfront(1)],[leftrear(2) leftfront(2)],'k','LineWidth',2)
    
    axis([0 1000 0 1000])
    axis square
    grid on
    pause(deltat/10)
    hold off
end

% figure()
% plot(positions(:,1),positions(:,2),'DisplayName',num2str(step(j)))
% hold on
