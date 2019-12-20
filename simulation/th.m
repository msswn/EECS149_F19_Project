function theta = th(xc,yc,r,xs,ys,xe,ye)

u = [r 0];
vs = [xs-xc, ys-yc];
ve = [xe-xc, ye-yc];
ths = acos(dot(u,vs)/(norm(u)*norm(vs)));
the = acos(dot(u,ve)/(norm(u)*norm(ve)));
% if (ve(2) < 0)
%     ths = 2*pi-ths;
%     the = 2*pi-the;
% end

theta = abs(the-ths);

end