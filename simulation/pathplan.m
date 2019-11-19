clear all
close all

% for St = 1:0.5:2
    S = 1;
    H = 0.75;
    l = 0.352;
    Rmin = l/tan(50*pi/180);
    
    k = (S*(H-2*Rmin)+sqrt(4*Rmin^2*(S^2+H^2)-16*Rmin^3*H))/(S^2-4*Rmin^2);
    m = Rmin*(1-sqrt(1+k^2));
    f1 = [S-k/sqrt(1+k^2)*Rmin, H-(1-1/sqrt(1+k^2))*Rmin];
    f2 = [k/sqrt(1+k^2)*Rmin, (1-1/sqrt(1+k^2))*Rmin];
    f = [f1; f2];
    
    xS = linspace(f1(1),f2(1));
    yS = k*xS + m;
    
    circle(S,H-Rmin,Rmin,S,H,f1(1),f1(2));
    hold on
    circle(0,Rmin,Rmin,0,0,f2(1),f2(2));
    hold on
    
    plot(0,0,'.r','MarkerSize',12)
    plot(S,H,'.r','MarkerSize',12)
    plot(f(:,1),f(:,2),'.k')
    plot(xS,yS,'k')
    axis equal
% end

