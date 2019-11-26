function h = circle(xc,yc,r,xs,ys,xe,ye)  

hold on
u = [r 0];
vs = [xs - xc, ys - yc];
ve = [xe - xc, ye - yc];
ths = acos(dot(u,vs)/(norm(u)*norm(vs)));
the = acos(dot(u,ve)/(norm(u)*norm(ve)));
if ve(2) < 0
    ths = 2*pi-ths;
    the = 2*pi-the;
end
th = ths:(the-ths)/50:the;

xunit = r * cos(th) + xc;
yunit = r * sin(th) + yc;
h = plot(xunit, yunit);
axis equal

end