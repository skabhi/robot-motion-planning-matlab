clear
clc
close all

figure(1)
title('Cartesian space')
axis("equal");
grid on;
axis([-10, 10, -10, 10]);
% axis('auto')
%% draw obstacle
rect1 = Obstacle('rectangle');
pose = [-6 -6 0]; size = [4,4];
rect1 = rect1.drawRect(pose,size,'red');

rect2 = Obstacle('rectangle');
pose = [-3 6 45]; size = [4,4];
rect2 = rect2.drawRect(pose,size,'cyan');

circle = Obstacle('circle');
circle = circle.drawCircle(5,6,3,'green');

tri = Obstacle('triangle');
% vertices = [3,6;3,4;6,5];
vertices = [1,6;3,4;6,5];
vertices(:,1) = vertices(:,1) + 2;
vertices(:,2) = vertices(:,2) - 8;
tri = tri.drawTriangle(vertices,'blue');
% tri.translate(0,-8);


%% 2r Manipulator
l1 = 4; l2 = 3;
base = [0 0];
arm = TwoR(base,l1,l2);
% arm = arm.draw2r(0,45);


%% C-space
step = 0.3;
theta1 = 0:step:2*pi;
theta2 = 0:step:2*pi;
for th1 = 1:length(theta1)
    for th2 = 1:length(theta2)
        figure(1)
        arm = arm.draw2r(theta1(th1),theta2(th2));
        pq = arm.linkpoints;
        for i=1:length(pq)
            incirc(i) = circle.isinside(pq(i,:));
            inrect1(i) = rect1.isinside(pq(i,:));
            inrect2(i) = rect2.isinside(pq(i,:));
            intri(i) = tri.isinside(pq(i,:));
        end
        cspacecirc(th1,th2) = sum(incirc);
        cspacerect1(th1,th2) = sum(inrect1);
        cspacerect2(th1,th2) = sum(inrect2);
        cspacetri(th1,th2) = sum(intri);
    end
end

cspace = cspacerect1 + cspacerect2 + cspacecirc + cspacetri;

figure(2)
axis("equal");

for th1 = 1:length(theta1)
    for th2 = 1:length(theta2)
        figure(2)
        if(cspace(th1,th2) == 0)
            plot(theta1(th1),theta2(th2),'*','Color','black');
            hold on;
        elseif(cspace(th1,th2) ~= 0 && cspacerect1(th1,th2) ~= 0)
            plot(theta1(th1),theta2(th2),'*','Color',rect1.color);
            hold on;
        elseif(cspace(th1,th2) ~= 0 && cspacerect2(th1,th2) ~= 0)
            plot(theta1(th1),theta2(th2),'*','Color',rect2.color);
            hold on;
        elseif(cspace(th1,th2) ~= 0 && cspacecirc(th1,th2) ~= 0)
            plot(theta1(th1),theta2(th2),'*','Color',circle.color);
            hold on;
        elseif(cspace(th1,th2) ~= 0 && cspacetri(th1,th2) ~= 0)
            plot(theta1(th1),theta2(th2),'*','Color',tri.color);
            hold on;
        else
            hold on;
        end
    end
end

title('C-space')
xlabel('\theta_1 (rad)'); ylabel('\theta_2 (rad)');
% axis([0, 7, 0, 7]);




