clear all
clc
close all

%% Define boundary
bnd = wsboundary;

f1 = figure(1);
f1.Position(1:2) = [1 50];
f1.Position(3:4) = [800 600];
title('RRT')
axis("equal");
grid on;
% bnd.drawboundary;
fig = gcf;
% axis([-10, 10, -10, 10]);
%% draw obstacle
Ofield = obstaclefield(bnd);
% Ofield.empty;
Ofield.setfield5;
% Ofield.draw;

%% RRT
qstart = [0,0]; qgoal = [7,-2];
delete(Ofield.starthandle);
% T = rrt(qstart,bnd,Ofield.obsfield);
T = rrt(Ofield.qstart,bnd,Ofield.obsfield);
T.setgoal(T.genqrand);
T.setgoal(qgoal);
T.bias = 0;
% T.checkqrand(2000)
T.drawtree;
% T.animate = 1;
T.shownodeindices = 1;
% T.buildtogoal;
% T.buildRRT(50);
% T.growandconnect(200);     % lavalle & kuffner
% T.shootandconnect(50);
T.movetogoal;
% T.shoot3branch;
T.drawtree;
% T.testgenqrandSC;
T.findpath;
f1 = figure(1);

%% Results
disp('')
disp('Number of Nodes: ')
disp(length(T.V))
disp('Path length: ')
disp(T.pathlength)
disp('Goal point: ')
disp(T.qgoal)
disp('Greedyness: ')
disp(T.bias*100)