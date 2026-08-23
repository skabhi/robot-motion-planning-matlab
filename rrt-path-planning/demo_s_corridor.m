clear
clc
close all

bnd = wsboundary;
figure('Name','RRT - S-corridor');
title('RRT - S-corridor');
axis equal
grid on

Ofield = obstaclefield(bnd);
Ofield.setfield9;
qgoal = [0,8];

delete(Ofield.starthandle);
T = rrt(Ofield.qstart,bnd,Ofield.obsfield);
T.setgoal(qgoal);
T.bias = 0;
T.step_size = 0.5;
T.drawtree;
T.growandconnect(200);
T.findpath;
T.drawtree;

fprintf('Nodes: %d\n',length(T.V));
fprintf('Path length: %.4f\n',T.pathlength);
fprintf('Goal: [%.2f, %.2f]\n',T.qgoal(1),T.qgoal(2));
