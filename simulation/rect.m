function [] = rect(L,W,center)

pt1 = center + [L/2, -W/2];
pt2 = center + [L/2, W/2];
pt3 = center + [-L/2, +W/2];
pt4 = center + [-L/2, -W/2];

plot([pt1(1),pt2(1)],[pt1(2),pt2(2)],'k');
hold on
plot([pt2(1),pt3(1)],[pt2(2),pt3(2)],'k');
plot([pt3(1),pt4(1)],[pt3(2),pt4(2)],'k');
plot([pt4(1),pt1(1)],[pt4(2),pt1(2)],'k');
