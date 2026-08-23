clear
clc
close all

bnd = wsboundary;
figure('Name','Artificial potential-field guidance');
title('Artificial potential-field guidance');
xlabel('X');
ylabel('Y');
axis equal
grid on

Ofield = obstaclefield(bnd);
Ofield.setfield1;
qstart = [0,0];
qgoal = [9.5,9.5];
Ofield.setgoal(qgoal);
Ofield.validqstart(qstart);

robot = pointrobot(Ofield);
robot.alpha = 0.025/Ofield.zeta;
robot.movetogoal;
robot.drawpath('g');

fprintf('Trajectory points: %d\n',size(robot.path,1));
fprintf('Final goal distance: %.6f\n',norm(robot.pose-Ofield.qgoal));

