function [X,Y,vertex] = Rectangle(pose,size,color)
% pose = [xpose ypose theta(deg)]
% size = [width height]
x = pose(1); y = pose(2); theta = pose(3);
w = size(1); h = size(2);

R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
vertex.v1 = R*[x,y]';
vertex.v2 = R*[x+w,y]';
vertex.v3 = R*[x+w,y+h]';
vertex.v4 = R*[x,y+h]';
hold on;
plot([vertex.v1(1) vertex.v2(1)],[vertex.v1(2) vertex.v2(2)],'LineWidth',2,'Color','black')
hold on;
plot([vertex.v2(1) vertex.v3(1)],[vertex.v2(2) vertex.v3(2)],'LineWidth',2,'Color','black')
hold on;
plot([vertex.v3(1) vertex.v4(1)],[vertex.v3(2) vertex.v4(2)],'LineWidth',2,'Color','black')
hold on;
plot([vertex.v4(1) vertex.v1(1)],[vertex.v4(2) vertex.v1(2)],'LineWidth',2,'Color','black')

xPatch = [vertex.v1(1) vertex.v2(1) vertex.v3(1) vertex.v4(1)];
yPatch = [vertex.v1(2) vertex.v2(2) vertex.v3(2) vertex.v4(2)];
patch(xPatch,yPatch,color)
hold off;

t = linspace(0,1,100);

for i = 1:length(t)
    Line.l1(i,1) = x + t(i)*w;
    Line.l1(i,2) = y;
    temp = R*[Line.l1(i,1); Line.l1(i,2)];
    Line.l1(i,1) = temp(1);
    Line.l1(i,2) = temp(2);

    Line.l2(i,1) = x + w;
    Line.l2(i,2) = y + t(i)*h;
    temp = R*[Line.l2(i,1); Line.l2(i,2)];
    Line.l2(i,1) = temp(1);
    Line.l2(i,2) = temp(2);

    Line.l3(i,1) = x + (1 - t(i))*w;
    Line.l3(i,2) = y + h;
    temp = R*[Line.l3(i,1); Line.l3(i,2)];
    Line.l3(i,1) = temp(1);
    Line.l3(i,2) = temp(2);

    Line.l4(i,1) = x;
    Line.l4(i,2) = y + (1 - t(i))*h;
    temp = R*[Line.l4(i,1); Line.l4(i,2)];
    Line.l4(i,1) = temp(1);
    Line.l4(i,2) = temp(2);
end

X = [Line.l1(:,1); Line.l2(:,1); Line.l3(:,1); Line.l4(:,1)];
Y = [Line.l1(:,2); Line.l2(:,2); Line.l3(:,2); Line.l4(:,2)];

end