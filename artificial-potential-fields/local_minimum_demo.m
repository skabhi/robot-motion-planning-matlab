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
Ofield.obsfield(3).Value.pose = [7,5.4];

%% APF
qstart = [0,0]; qgoal = [9.5,9.5];
Ofield.setgoal(qgoal);
Ofield.validqstart(qstart);
% Ofield.drawgoalVF;
% Ofield.drawRepVF;
% Ofield.drawVF;
Ofield.drawPF;
%% Simulation
% Ofield.zeta = 10;
robot = pointrobot(Ofield);
robot.LMtol = 0.01;
robot.alpha = 0.025/Ofield.zeta;
robot.movetogoal;
robot.drawpath('g');

%% Plot
% run this section separately
figure
plot(robot.gradnorm,'b');
grid on;
title('Norm of the total gradient')
xlabel('Iteration');
ylabel('$ \|\|  \nabla U  \|\| $','fontsize',14,'interpreter','latex');