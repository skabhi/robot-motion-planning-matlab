classdef edge
    properties
        node1       % first node connecting this edge
        node2       % another node connecting this edge
        Npoints = 1000;  % Number of discretizing points
    end
    methods
        % Constructor
        function obj = edge(n1,n2)
            obj.node1 = n1;
            obj.node2 = n2;
        end
        % draw this edge on current figure
        function  h = drawedge(obj)
            hold on;
            h = plot([obj.node1.pose(1),obj.node2.pose(1)],...
                [obj.node1.pose(2),obj.node2.pose(2)],"color",[0.8 0 0],"LineWidth",0.5);
            hold off;
        end
        % Check if edge e is intersecting with this edge
        % 1 = intersect
        % 0 = don't intersect
        function intersect = checkintersection(obj,e)
            p1 =  Point(obj.node1.pose(1), obj.node1.pose(2));
            q1 =  Point(obj.node2.pose(1), obj.node2.pose(2));
            p2 =  Point(e.node1.pose(1), e.node1.pose(2));
            q2 =  Point(e.node2.pose(1), e.node2.pose(2));

            intersect = doIntersect(p1, q1, p2, q2);
        end
        % discretize this edge
        function points = discretise(obj)
            %             d = round(obj.bnd.xmax*norm(qnear - qnew));
            %             npoints = 20*d;
            %             if(npoints < 10)
            %                 npoints = 10;
            %             end
            npoints = obj.Npoints;
            x = linspace(obj.node1.pose(1),obj.node2.pose(1),npoints);
            y = linspace(obj.node1.pose(2),obj.node2.pose(2),npoints);
            points = [x' y'];
        end
        % Check collision of this edge with obstacle field obs
        function collision = iscolliding(obj,obs)
            collision = 0;
            edgepoint = obj.discretise;
            for i = 1:size(edgepoint,1)
                collision = collision + obs.isinside(...
                    [edgepoint(i,1),edgepoint(i,2)]);
            end
        end
        % Length of this edge
        function L = edgelength(obj)
            L = norm(obj.node1.pose - obj.node2.pose);
        end
    end
end