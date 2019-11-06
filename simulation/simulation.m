close all
clear all

% wheelbase
l = 1.524;
% Track width
trackwidth = 1.27;
% Rear overhang
Lb = 0.5;
% Front overhang
Lf = 0.5;

v = -1;
tEnd = 10;
p0 = [0 0];

% step = [1 0.1 0.01 0.001 0.0001];

t = 0:0.1:tEnd;
deltat = t(2)-t(1);
numTimeSteps = length(t);

thetas = zeros(numTimeSteps,1);
steeringAngle = zeros(numTimeSteps,1);
steeringAngle(1:33) = 30*(pi/180);
steeringAngle(66:end) = -30*(pi/180);

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
    r = positions(i,:);
    phi = steeringAngle(i);
    
    Rtheta = [cos(thetas(i)) -sin(thetas(i)); sin(thetas(i)) cos(thetas(i))];
    Rphi = [cos(thetas(i)+phi) -sin(thetas(i)+phi); sin(thetas(i)+phi) cos(thetas(i)+phi)];
    
    a = (Rtheta*[l; 0])';
    f = r+a;
    
    b = (Rphi*[0.4; 0])';
    c = (Rtheta*[0.4; 0])';
    
    % car body boundaries
    rlb = r+(Rtheta*[-Lb; trackwidth/2])';
    rrb = r+(Rtheta*[-Lb; -trackwidth/2])';
    flb = f+(Rtheta*[Lf; trackwidth/2])';
    frb = f+(Rtheta*[Lf; -trackwidth/2])';
    
    % tires
    fr = f + (Rtheta*[0; -trackwidth/2])';
    fl = f + (Rtheta*[0; trackwidth/2])';
    rl = r + (Rtheta*[0; trackwidth/2])';
    rr = r + (Rtheta*[0; -trackwidth/2])';
    tirecoords = [fr; fl; rl; rr];
    
    frr = fr-b;
    frf = fr+b;
    flr = fl-b;
    flf = fl+b;
    rlr = rl-c;
    rlf = rl+c;
    rrr = rr-c;
    rrf = rr+c;
    
    ff = f+b;
    fr = f-b;
    
    figure(1)
    plot([f(1) r(1)],[f(2) r(2)],'b')
    hold on
    plot([r(1) f(1)],[r(2) f(2)],'.r','MarkerSize',5)
    plot([frr(1) frf(1)],[frr(2) frf(2)],'k','LineWidth',2)
    plot([flr(1) flf(1)],[flr(2) flf(2)],'k','LineWidth',2)
    plot([rlr(1) rlf(1)],[rlr(2) rlf(2)],'k','LineWidth',2)
    plot([rrr(1) rrf(1)],[rrr(2) rrf(2)],'k','LineWidth',2)
    plot([rlb(1) rrb(1)],[rlb(2) rrb(2)],'k')
    plot([rlb(1) flb(1)],[rlb(2) flb(2)],'k')
    plot([flb(1) frb(1)],[flb(2) frb(2)],'k')
    plot([frb(1) rrb(1)],[frb(2) rrb(2)],'k')
    axis([-5 15 -5 15])
    axis square
    grid on
    hold off
end

% figure()
% plot(positions(:,1),positions(:,2),'DisplayName',num2str(step(j)))
% hold on
