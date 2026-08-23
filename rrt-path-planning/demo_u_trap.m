clear
clc
close all

bnd = wsboundary;
figure('Name','RRT - U-shaped cul-de-sac');
title('RRT - U-shaped cul-de-sac');
axis equal
grid on

Ofield = obstaclefield(bnd);
Ofield.setfield8;
qgoal = [8,-6];

delete(Ofield.starthandle);
T = rrt(Ofield.qstart,bnd,Ofield.obsfield);
T.setgoal(qgoal);
T.bias = 0.05;
T.step_size = 0.5;
T.drawtree;
T.growandconnect(250);
T.findpath;
T.drawtree;

fprintf('Nodes: %d\n',length(T.V));
fprintf('Path length: %.4f\n',T.pathlength);
fprintf('Goal: [%.2f, %.2f]\n',T.qgoal(1),T.qgoal(2));
