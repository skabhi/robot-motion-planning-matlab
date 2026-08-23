classdef TwoR
    properties
        theta1
        theta2
        l1
        l2
        base
        x1
        y1
        x2
        y2
    end
    methods
        function obj = TwoR(base,l1,l2)
            obj.l1 = l1; obj.l2 = l2;
            obj.base = base;
        end

        function obj = draw2r(obj,theta1,theta2)
            c1 = cos(theta1); s1 = sin(theta1);
            c12 = cos(theta1 + theta2); s12 = sin(theta1 + theta2);

            obj.x1 = obj.base(1) + obj.l1*c1;
            obj.y1 = obj.base(2) + obj.l1*s1;

            obj.x2 = obj.x1 + obj.l2*c12;
            obj.y2 = obj.y1 + obj.l2*s12;

            hold on;
            plot([obj.base(1),obj.x1],[obj.base(2),obj.y1],"color",[0 0 0],"LineWidth",0.5);
            plot(obj.x1,obj.y1,'bo');
            hold on;
            plot([obj.x1,obj.x2],[obj.y1,obj.y2],"color",[0 0 0],"LineWidth",0.5);
            plot(obj.x2,obj.y2,'bo');
            hold off;

        end

        function pq = linkpoints(obj)
            npoints = 20;
            link1x = linspace(obj.base(1),obj.x1,npoints);
            link1y = linspace(obj.base(2),obj.y1,npoints);
            link2x = linspace(obj.x1,obj.x2,npoints);
            link2y = linspace(obj.y2,obj.y2,npoints);

            pq = [link1x' link1y'; link2x' link2y'];
        end

    end
end