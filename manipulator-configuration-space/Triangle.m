function [X,Y] = Triangle(vertices,color)
% vertices = [x1 y1; x2 y1; x3 y3]

vertex.v1 = [vertices(1,1),vertices(1,2)]';
vertex.v2 = [vertices(2,1),vertices(2,2)]';
vertex.v3 = [vertices(3,1),vertices(3,2)]';

hold on;
plot([vertex.v1(1) vertex.v2(1)],[vertex.v1(2) vertex.v2(2)],'LineWidth',2,'Color','black')
hold on;
plot([vertex.v2(1) vertex.v3(1)],[vertex.v2(2) vertex.v3(2)],'LineWidth',2,'Color','black')
hold on;
plot([vertex.v3(1) vertex.v1(1)],[vertex.v3(2) vertex.v1(2)],'LineWidth',2,'Color','black')


xPatch = [vertex.v1(1) vertex.v2(1) vertex.v3(1)];
yPatch = [vertex.v1(2) vertex.v2(2) vertex.v3(2)];
patch(xPatch,yPatch,color)
hold off;

t = linspace(0,1,100);

for i = 1:length(t)
    Line.l1(i,1) = vertex.v1(1) + t(i)*(vertex.v2(1) - vertex.v1(1));
    Line.l1(i,2) = vertex.v1(2) + t(i)*(vertex.v2(2) - vertex.v1(2));

    Line.l2(i,1) = vertex.v2(1) + t(i)*(vertex.v3(1) - vertex.v2(1));
    Line.l2(i,2) = vertex.v2(2) + t(i)*(vertex.v3(2) - vertex.v2(2));

    Line.l3(i,1) = vertex.v3(1) + t(i)*(vertex.v1(1) - vertex.v3(1));
    Line.l3(i,2) = vertex.v3(2) + t(i)*(vertex.v1(2) - vertex.v3(2));
end

X = [Line.l1(:,1); Line.l2(:,1); Line.l3(:,1)];
Y = [Line.l1(:,2); Line.l2(:,2); Line.l3(:,2)];

end