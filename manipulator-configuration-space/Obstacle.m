classdef Obstacle
    properties
        pose
        Size
        vertices
        type
        x
        y
        R
        color
    end
    methods
        function obj = Obstacle(type)
            obj.type = type;
            if(strcmp(type,'Rectangle') | strcmp(type,'rectangle'))
                obj.pose = [0,0,0];
                obj.Size = [1,1];
            elseif(strcmp(type,'Circle') | strcmp(type,'circle'))
                %                 obj.pose = [0,0];
                %                 obj.size = 1;
            elseif(strcmp(type,'Triangle') | strcmp(type,'triangle'))
            end
        end

        function obj = drawRect(obj,pose,size,color)
            obj.pose = pose;
            obj.Size = size;
            [obj.x, obj.y,vertex] = Rectangle(pose,size,color);
            obj.vertices = [vertex.v1(1) vertex.v1(2);...
                vertex.v2(1) vertex.v2(2); vertex.v3(1) vertex.v3(2);...
                vertex.v4(1) vertex.v4(2)];

            obj.color = color;
            obj = rotVertex(obj,obj.vertices);
        end

        function obj = drawCircle(obj,x,y,r,color)
            hold on
            th = 0:pi/50:2*pi;
            obj.x = r * cos(th) + x;
            obj.y = r * sin(th) + y;
            plot(obj.x, obj.y);
            patch(obj.x,obj.y,color)
            hold off

            obj.pose = [x,y];
            obj.Size = r;
            obj.color = color;

        end

        function obj = drawTriangle(obj,vertices,color)
            [obj.x, obj.y] = Triangle(vertices,color);
            obj.vertices = vertices;
            obj = rotVertexTri(obj);
            obj.color = color;
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
            end
        end
    end
end