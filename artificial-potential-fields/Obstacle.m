classdef Obstacle
    properties
        pose
        Size
        vertices
        type
        x
        y
        npoint = 100; % number of boundary points
        R
        color
    end
    methods
        function obj = Obstacle(type)
            obj.type = type;
            if(strcmp(type,'Rectangle') || strcmp(type,'rectangle'))
                obj.pose = [0,0,0];
                obj.Size = [1,1];
            elseif(strcmp(type,'Circle') || strcmp(type,'circle'))
                %                 obj.pose = [0,0];
                %                 obj.size = 1;
            elseif(strcmp(type,'Triangle') || strcmp(type,'triangle'))
            end
        end

        function draw(obj)
            if(strcmp(obj.type,'Rectangle') || strcmp(obj.type,'rectangle'))
                obj.drawRect;
            elseif(strcmp(obj.type,'Circle') || strcmp(obj.type,'circle'))
                obj.drawCircle;
            elseif(strcmp(obj.type,'Triangle') || strcmp(obj.type,'triangle'))
                obj.drawTriangle;
            elseif(strcmp(obj.type,'Empty') || strcmp(obj.type,'empty'))
            else
                error('wrong obstacle type')
            end
        end
        % draw distance field
        function drawdistfield(obj)
            x = linspace(0,10);
            y = linspace(0,10);
            [X,Y] = meshgrid(x,y);
            D = obj.distfield(X,Y);
            obj.draw;
            hold on
            contour(X,Y,D)
            hold off
        end
        % make column vector
        function cq = makecolumn(obj,q)
            if(~iscolumn(q))
                cq = q';
            else
                cq = q;
            end
        end
        % Euclidean distance from obstacle
        function [d,obspoint] = distancefrom(obj,q)
            if(strcmp(obj.type,'Empty') || strcmp(obj.type,'empty'))
                d = -1;
                obspoint = q;
            elseif(strcmp(obj.type,'Circle') || strcmp(obj.type,'circle'))
                q = obj.makecolumn(q);
                c = obj.makecolumn(obj.pose);
                qc = q - c;
                theta = atan2(qc(2),qc(1));           
                obspoint = c + obj.Size*[cos(theta); sin(theta)];
                d = norm(q-c) - obj.Size;
            else
                q = obj.makecolumn(q);
                for i = 1:length(obj.x)
                    b(:,i) = [obj.x(i); obj.y(i)];
                    dist(i) = norm(b(:,i) - q);
                end
                [d,i] = min(dist);
                obspoint = b(:,i);
            end
        end
        % Distance function (input is meshgrid)
        function D = distfield(obj,X,Y)
            for i = 1:size(X,1)
                for j = 1:size(X,2)
                    q = [X(i,j); Y(i,j)];
                    D(i,j) = obj.distancefrom(q);
                end
            end
        end
        % Rectangle
        function obj = setRectangle(obj,pose,size,color)
            % pose = [xpose ypose theta(deg)]
            % size = [width height]
            obj.pose = pose;
            obj.Size = size;
            obj.color = color;

            x = pose(1); y = pose(2); theta = pose(3);
            w = size(1); h = size(2);

            R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
            obj.vertices(1,:) = R*[x,y]';
            obj.vertices(2,:) = R*[x+w,y]';
            obj.vertices(3,:) = R*[x+w,y+h]';
            obj.vertices(4,:) = R*[x,y+h]';

            t = linspace(0,1,obj.npoint);

            for i = 1:length(t)
                Line.l1(i,1) = x + t(i)*w;
                Line.l1(i,2) = y;
                [Line.l1(i,1); Line.l1(i,2)];
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

            obj.x = [Line.l1(:,1); Line.l2(:,1); Line.l3(:,1); Line.l4(:,1)];
            obj.y = [Line.l1(:,2); Line.l2(:,2); Line.l3(:,2); Line.l4(:,2)];
            obj = rotVertex(obj,obj.vertices);

        end

        function drawRect(obj)
            v1 = obj.vertices(1,:);
            v2 = obj.vertices(2,:);
            v3 = obj.vertices(3,:);
            v4 = obj.vertices(4,:);
            hold on;
            plot([v1(1) v2(1)],[v1(2) v2(2)],...
                'LineWidth',2,'Color','black')
            hold on;
            plot([v2(1) v3(1)],[v2(2) v3(2)],...
                'LineWidth',2,'Color','black')
            hold on;
            plot([v3(1) v4(1)],[v3(2) v4(2)],...
                'LineWidth',2,'Color','black')
            hold on;
            plot([v4(1) v1(1)],[v4(2) v1(2)],...
                'LineWidth',2,'Color','black')

            xPatch = [v1(1) v2(1) v3(1) v4(1)];
            yPatch = [v1(2) v2(2) v3(2) v4(2)];
            patch(xPatch,yPatch,obj.color)
            hold off;
        end
        % Circle
        function obj = setCircle(obj,x,y,r,color)
            th = 0:pi/50:2*pi;
            obj.x = r * cos(th) + x;
            obj.y = r * sin(th) + y;

            obj.pose = [x,y];
            obj.Size = r;
            obj.color = color;
        end

        function drawCircle(obj)
            hold on
            plot(obj.x, obj.y);
            patch(obj.x,obj.y,obj.color)
            hold off
        end
        % Triangle
        function obj = setTriangle(obj,vertices,color)
            % vertices = [x1 y1; x2 y1; x3 y3]
            obj.vertices = vertices;
            obj.color = color;

            v1 = [vertices(1,1),vertices(1,2)]';
            v2 = [vertices(2,1),vertices(2,2)]';
            v3 = [vertices(3,1),vertices(3,2)]';

            t = linspace(0,1,obj.npoint);

            for i = 1:length(t)
                Line.l1(i,1) = v1(1) + t(i)*(v2(1) - v1(1));
                Line.l1(i,2) = v1(2) + t(i)*(v2(2) - v1(2));

                Line.l2(i,1) = v2(1) + t(i)*(v3(1) - v2(1));
                Line.l2(i,2) = v2(2) + t(i)*(v3(2) - v2(2));

                Line.l3(i,1) = v3(1) + t(i)*(v1(1) - v3(1));
                Line.l3(i,2) = v3(2) + t(i)*(v1(2) - v3(2));
            end

            obj.x = [Line.l1(:,1); Line.l2(:,1); Line.l3(:,1)];
            obj.y = [Line.l1(:,2); Line.l2(:,2); Line.l3(:,2)];

        end

        function drawTriangle(obj)
            v1 = [obj.vertices(1,1),obj.vertices(1,2)]';
            v2 = [obj.vertices(2,1),obj.vertices(2,2)]';
            v3 = [obj.vertices(3,1),obj.vertices(3,2)]';

            hold on;
            plot([v1(1) v2(1)],[v1(2) v2(2)],'LineWidth',2,'Color','black')
            hold on;
            plot([v2(1) v3(1)],[v2(2) v3(2)],'LineWidth',2,'Color','black')
            hold on;
            plot([v3(1) v1(1)],[v3(2) v1(2)],'LineWidth',2,'Color','black')


            xPatch = [v1(1) v2(1) v3(1)];
            yPatch = [v1(2) v2(2) v3(2)];
            patch(xPatch,yPatch,obj.color)
            hold off;
        end

        function translate(obj,x,y)
            if(strcmp(obj.type,'Triangle') | strcmp(obj.type,'triangle'))
                obj.vertices(:,1) = obj.vertices(:,1) + x;
                obj.vertices(:,2) = obj.vertices(:,2) + y;
                drawTriangle(obj.vertices);
            end

        end

        function obj = rotVertex(obj,vertices)
            s = size(vertices);
            theta = obj.pose(3);
            for i = 1:s(1)
                fname = ['R',num2str(i)];
                Rot.(fname) = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
                theta = theta + 90;
            end
            obj.R = Rot;
        end

        function obj = rotVertexTri(obj)
            s = size(obj.vertices);

            for i = 1:s(1)
                fname = ['R',num2str(i)];
                v1 = obj.vertices(i,:);
                if(i==3)
                    v2 = obj.vertices(1,:);
                else
                    v2 = obj.vertices(i+1,:);
                end
                theta = atan2d((v2(2) - v1(2)),(v2(1) - v1(1)));
                Rot.(fname) = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
            end
            obj.R = Rot;
        end


        function in = isinside(obj,pq)
            % in = 1, if query point pq is inside obstacle obj
            % in = 0, if query point pq is outside obstacle obj
            if(~iscolumn(pq))
                pq = pq';
            end
            if(strcmp(obj.type,'Circle') || strcmp(obj.type,'circle'))
                if(norm(obj.pose' - pq) <= obj.Size)
                    in = 1;
                else
                    in = 0;
                end

            elseif(strcmp(obj.type,'Rectangle') || strcmp(obj.type,'rectangle'))
                s = size(obj.vertices);
                in = 1;
                for i = 1:s(1)
                    fname = ['R',num2str(i)];
                    pqv = (obj.R.(fname))'*(pq - obj.vertices(i,:)');
                    %                     disp(pqv)
                    if(pqv(1) < 0 || pqv(2) < 0)
                        in = 0;
                    end
                end

            elseif(strcmp(obj.type,'Triangle') || strcmp(obj.type,'triangle'))
                s = size(obj.vertices);
                in = 1;
                for i = 1:s(1)

                    if(i == 1)
                        i2 = 2; i3 = 3;
                    elseif(i == 2)
                        i2 = 3; i3 = 1;
                    elseif(i == 3)
                        i2 = 1; i3 = 2;
                    end

                    T = [(obj.vertices(i2,:)' - obj.vertices(i,:)') ...
                        (obj.vertices(i3,:)' - obj.vertices(i,:)')];
                    pqv = inv(T)*(pq - obj.vertices(i,:)');

                    if(pqv(1) < 0 || pqv(2) < 0)
                        in = 0;
                    end
                end
            elseif(strcmp(obj.type,'Empty') || strcmp(obj.type,'empty'))
                in = 0;
            end
        end
    end
end