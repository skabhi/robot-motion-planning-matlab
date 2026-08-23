classdef obstaclefield < handle
    properties
        obsfield        % Obstacle object array
        qstart          % Ideal start position for obsfield
        bnd             % boundary object
        Qstar = 0.5;    % threshold for repulsive field
        qstar = 1;      % threshold for attractive field
        eta = 5;        % Repulsive field scale
        zeta = 5;       % Attractive field scale
        qgoal;          % Goal point
    end
    methods
        function obj = obstaclefield(bnd)
            obj.bnd = bnd;
        end
        % set goal configuration
        function obj = setgoal(obj,goal)
            goalisinsideobs = obj.obsfield.isinside(goal);
            goalisinsidebnd = obj.bnd.isinside(goal);
            if(~goalisinsideobs && goalisinsidebnd)
                obj.qgoal = obj.makecolumn(goal);
            else
                if(goalisinsideobs)
                    error('Goal is inside the obstacle');
                elseif(~goalisinsidebnd)
                    error('Goal is outside the boundary');
                end
            end
            hold on
            plot(obj.qgoal(1),obj.qgoal(2),'.',...
                'color',[1 0 1],'MarkerSize',30);
            hold off
        end
        % Goal Attractive field
        function [delU,U] = goalfield(obj,q)
            q = obj.makecolumn(q);
            d = norm(q - obj.qgoal);
            if(~obj.obsfield.isinside(q))
                if(d < obj.qstar)
                    % Quadratic
                    delU = obj.zeta*(obj.qgoal - q);
                    U = 0.5*obj.zeta*d^2;
                else
                    % Conical
                    delU = obj.zeta/d*(obj.qgoal - q);
                    U = obj.zeta*d;
                end
            else
                delU = [0;0];
                U = 0;
            end
        end
        % empty obstacle field
        function obj = empty(obj)
            obj.bnd = obj.bnd.set(-10,10,-10,10);
            obj.bnd.drawboundary;
            emp = Obstacle('empty');
            obj.obsfield = ObjectArray(emp);

            obj.qstart = [0,0];
        end
        % Obstacle field
        function obj = setfield1(obj)
            %             bnd = bnd.set(-10,10,-10,10);
            obj.bnd.drawboundary;

            rect = Obstacle('rectangle');
            pose = [2 2.5 0]; size = [2,2];
            rect = rect.setRectangle(pose,size,'red');

            circle = Obstacle('circle');
            circle = circle.setCircle(7,4.5,1,'red');

            tri = Obstacle('triangle');
            vertices = [3,8;2,6;4,6];
            vertices(:,1) = vertices(:,1) + 2;
            vertices(:,2) = vertices(:,2) - 0;
            tri = tri.setTriangle(vertices,'red');

            obj.obsfield = ObjectArray([rect,tri,circle]);

            %             obj.validqstart([1,1]);

            obj.draw;
        end
        % Obstacle field
        function obj = setfield2(obj)
            obj.bnd = obj.bnd.set(-10,10,-10,10);
            obj.bnd.drawboundary;

            side = 8.5;

            sq1 = Obstacle('rectangle');
            pose = [-9 0.5 0]; size = [side,side];
            sq1 = sq1.setRectangle(pose,size,'blue');

            sq2 = Obstacle('rectangle');
            pose = [0.5 0.5 0]; size = [side,side];
            sq2 = sq2.setRectangle(pose,size,'blue');

            sq3 = Obstacle('rectangle');
            pose = [-9 -9 0]; size = [side,side];
            sq3 = sq3.setRectangle(pose,size,'blue');

            sq4 = Obstacle('rectangle');
            pose = [0.5 -9 0]; size = [side,side];
            sq4 = sq4.setRectangle(pose,size,'blue');

            obj.obsfield = ObjectArray([sq1,sq2,sq3,sq4]);

            obj.validqstart([-9.9,0]);

            obj.draw;

        end
        % Obstacle field
        function obj = setfield3(obj)
            obj.bnd = obj.bnd.set(-10,10,-10,10);
            obj.bnd.drawboundary;

            Size = 0.5;

            wall1 = Obstacle('rectangle');
            pose = [-3.5 -3 0]; size = [Size,13];
            wall1 = wall1.setRectangle(pose,size,'blue');

            wall2 = Obstacle('rectangle');
            pose = [-3.5 -10 0]; size = [Size,6];
            wall2 = wall2.setRectangle(pose,size,'blue');

            wall3 = Obstacle('rectangle');
            pose = [3 4 0]; size = [Size,6];
            wall3 = wall3.setRectangle(pose,size,'blue');

            wall4 = Obstacle('rectangle');
            pose = [3 -10 0]; size = [Size,12];
            wall4 = wall4.setRectangle(pose,size,'blue');

            obj.obsfield = ObjectArray([wall1,wall2,wall3,wall4]);

            obj.validqstart([0,0]);

            obj.draw;
        end
        % Obstacle field
        function obj = setfield4(obj)
            obj.bnd = obj.bnd.set(-10,10,-10,10);
            obj.bnd.drawboundary;

            tri = Obstacle('triangle');
            vertices = [-8,-10;8,-10;0,0];
            tri = tri.setTriangle(vertices,'blue');


            obj.obsfield = ObjectArray([tri]);

            obj.validqstart([-9,-9]);

            obj.draw;

        end
        % Obstacle field
        function obj = setfield5(obj)
            obj.bnd = obj.bnd.set(-10,10,-10,10);
            obj.bnd.drawboundary;

            block1 = Obstacle('rectangle');
            pose = [-3.5 0.5 0]; size = [7,9.5];
            block1 = block1.setRectangle(pose,size,'blue');

            block2 = Obstacle('rectangle');
            pose = [-3.5 -10 0]; size = [7,9.5];
            block2 = block2.setRectangle(pose,size,'blue');


            obj.obsfield = ObjectArray([block1,block2]);

            obj.validqstart([-9,-9]);

            obj.draw;
        end
        % Obstacle field
        function obj = setfield6(obj)
            obj.bnd = obj.bnd.set(-10,10,-10,10);
            obj.bnd.drawboundary;

            wall1 = Obstacle('rectangle');
            pose = [-3 1.5 0]; size = [6,0.5];
            wall1 = wall1.setRectangle(pose,size,'blue');

            wall2 = Obstacle('rectangle');
            pose = [-3 -2 0]; size = [6,0.5];
            wall2 = wall2.setRectangle(pose,size,'blue');

            wall3 = Obstacle('rectangle');
            pose = [2.5 -1.5 0]; size = [0.5,3];
            wall3 = wall3.setRectangle(pose,size,'blue');


            obj.obsfield = ObjectArray([wall1,wall2,wall3]);

            obj.validqstart([0,0]);

            obj.draw;
        end

        % check if start position is valid
        function obj = validqstart(obj,qstart)
            if(obj.obsfield.isinside(qstart))
                error('Start position is inside obstacle');
            end
            obj.qstart = qstart;
            hold on;
            w = 0.3;h = 0.3;
            rectangle('Position',[obj.qstart(1)-w/2,obj.qstart(2)-h/2,w,h],...
                'FaceColor',[1 1 0],'EdgeColor','black','LineWidth',0.5);
            hold off;
        end
        % draw distace field
        function Dmin = drawdistfield(obj)
            x = linspace(obj.bnd.xmin,obj.bnd.xmax);
            y = linspace(obj.bnd.ymin,obj.bnd.ymax);
            [X,Y] = meshgrid(x,y);
            for i = 1:length(obj.obsfield)
                D(:,:,i) = obj.obsfield(1).Value.distfield(X,Y);
            end

            for i = 1:size(X,1)
                for j = 1:size(X,2)
                    Dmin(i,j) = min(D(i,j,:));
                end
            end
            %             obj.draw;
            hold on
            contour(X,Y,Dmin)
            hold off
        end
        % Repulsive field
        function [DelU, U] = repfield(obj,q)
            DUsum = [0;0]; Usum = 0;
            q = obj.makecolumn(q);
            for i = 1:length(obj.obsfield)
                [D,point] = obj.obsfield(i).Value.distancefrom(q);
                if(D < obj.Qstar && ~obj.obsfield(i).Value.isinside(q))
                    DUsum = DUsum + obj.eta*(1/D - 1/obj.Qstar)/D*(q-point);
                    Usum = Usum + 0.5*obj.eta*(1/D - 1/obj.Qstar);
                elseif(obj.obsfield(i).Value.isinside(q))
                    Usum = Usum + 3000;
                end
            end
            DelU = DUsum;
            U = Usum;
        end
        % draw distace field
        function drawRepVF(obj)
            x = linspace(obj.bnd.xmin,obj.bnd.xmax);
            y = linspace(obj.bnd.ymin,obj.bnd.ymax);
            [X,Y] = meshgrid(x,y);
            for i = 1:size(X,1)
                for j = 1:size(X,2)
                    q = [X(i,j); Y(i,j)];
                    DelU = obj.repfield(q);
                    U(i,j) = DelU(1);
                    V(i,j) = DelU(2);
                end
            end
            hold on
            quiver(X,Y,U,V,'r');
            hold off
        end
        % draw goal vector field
        function drawgoalVF(obj)
            x = linspace(obj.bnd.xmin,obj.bnd.xmax);
            y = linspace(obj.bnd.ymin,obj.bnd.ymax);
            [X,Y] = meshgrid(x,y);
            for i = 1:size(X,1)
                for j = 1:size(X,2)
                    q = [X(i,j); Y(i,j)];
                    DelU = obj.goalfield(q);
                    U(i,j) = DelU(1);
                    V(i,j) = DelU(2);
                end
            end
            hold on
            quiver(X,Y,U,V,'g');
            hold off
        end
        % draw Total vector field
        function drawVF(obj)
            x = linspace(obj.bnd.xmin,obj.bnd.xmax);
            y = linspace(obj.bnd.ymin,obj.bnd.ymax);
            [X,Y] = meshgrid(x,y);
            for i = 1:size(X,1)
                for j = 1:size(X,2)
                    q = [X(i,j); Y(i,j)];
                    DelU = obj.goalfield(q) + obj.repfield(q);
                    DelU = DelU/norm(DelU);
                    U(i,j) = DelU(1);
                    V(i,j) = DelU(2);
                end
            end
            hold on
            quiv = quiver(X,Y,U,V,1);
            quiv.Color = 'cyan';
            hold off
        end
        % draw Total potential field
        function drawPF(obj)
            x = linspace(obj.bnd.xmin,obj.bnd.xmax);
            y = linspace(obj.bnd.ymin,obj.bnd.ymax);
            [X,Y] = meshgrid(x,y);
            for i = 1:size(X,1)
                for j = 1:size(X,2)
                    q = [X(i,j); Y(i,j)];
                    [DelU,urep] = obj.repfield(q);
                    [DelU,uatt] = obj.goalfield(q);
                    U(i,j) = uatt + urep/50;
                end
            end
            figure
            surf(X,Y,U);
            title('Potential field')
            xlabel('X'), ylabel('Y'); zlabel('Z');
            %             quiv.Color = 'cyan';
        end
        % draw obstacle field
        function draw(obj)
            for i = 1:length(obj.obsfield)
                obj.obsfield(i).Value.draw;
            end
        end
        % make column vector
        function cq = makecolumn(obj,q)
            if(~iscolumn(q))
                cq = q';
            else
                cq = q;
            end
        end
    end
end