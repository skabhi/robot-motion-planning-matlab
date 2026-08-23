clear all
clc
close all

%% Define boundary
bnd = wsboundary;

f1 = figure(1);
f1.Position(1:2) = [1 50];
f1.Position(3:4) = [800 600];
title('Workspace')
xlabel('X'), ylabel('Y');
axis("equal");
grid on;
% bnd.drawboundary;.
fig = gcf;
% axis([-10, 10, -10, 10]);
%% draw obstacle

Ofield = obstaclefield(bnd);
% Ofield = Ofield.empty;
Ofield.setfield1;
% Ofield.draw;
% Ofield.drawdistfield;

%% APF
qstart = [0,0]; qgoal = [9.5,9.5];
Ofield.setgoal(qgoal);
Ofield.validqstart(qstart);
% Ofield.drawgoalVF;
% Ofield.drawRepVF;
% Ofield.drawVF;
% Ofield.drawPF;

%% Simulation
% Ofield.zeta = 10;
robot = pointrobot(Ofield);
robot.alpha = 0.025/Ofield.zeta;
robot.movetogoal;
robot.drawpath('b');

%% Robot 2 simulation
Ofield.zeta = 10;
robot2 = pointrobot(Ofield);
robot2.alpha = 0.025/Ofield.zeta;
robot2.movetogoal;
robot2.drawpath('r');

%% Robot 3 simulation
Ofield.zeta = 2;
robot3 = pointrobot(Ofield);
robot3.alpha = 0.025/Ofield.zeta;
robot3.movetogoal;
robot3.drawpath('g');
