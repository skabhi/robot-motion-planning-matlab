classdef rrt < handle
    properties
        qroot               % position of root of tree
        qgoal               % goal position
        goalradius = 0.3;   % goal reaching radius
        bnd                 % boundary object
        obsfield            % Obstacle field object
        V                   % Nodes object array
        E                   % Edges object array
        treehandle
        nodetexthndl
        thetarange = 220;
        path                % Object array of edges connecting goal to start
        bias = 0.1;         % Exploration bias
        step_size = 0.3;    % Edge length
        animate = 0;        % set 1 if animation is required
        shownodeindices = 0;    % set 1 to show nodes ids
        gennum
        nochildnodes
    end
    methods
        % Constructor with workspace boundary and obstacle as input
        function obj = rrt(q0,bnd,obs)
            if(~iscolumn(q0))
                q0 = q0';
            end
            obj.qroot = q0;
            obj.bnd = bnd;
            obj.obsfield = obs;
            startnode = node(q0);
            startnode.index = 1;
            startnode.pindex = 0;
            %             startnode = startnode.setpose(q0);
            obj.V = ObjectArray(startnode);
            obj.E = ObjectArray([]);
            obj.E(1) = [];
            obj.path = ObjectArray([]);
            obj.path(1) = [];

            obj.gennum = 0;
            obj.nochildnodes = startnode.index;
        end
        % update boundary
        function updatebnd(obj,bnd)
            obj.bnd = bnd;
        end
        % set goal configuration
        function setgoal(obj,goal)
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
            %             hold on
            %             plot(obj.qgoal(1),obj.qgoal(2),'.',...
            %                 'color',[1 0 1],'MarkerSize',30);
            %             hold off
        end
        % genrate random configuration qrand with exploration bias
        % using inverse transform sampling
        function qrand = genqrandbiased(obj)
            free = 0;
            while(~free)
                %                 qrand = [rand(1)*(obj.bnd.xmax - obj.bnd.xmin) + obj.bnd.xmin;...
                %                     rand(1)*(obj.bnd.ymax - obj.bnd.ymin) + obj.bnd.ymin];
                normgoal = [(obj.qgoal(1) - obj.bnd.xmin)/(obj.bnd.xmax...
                    - obj.bnd.xmin); (obj.qgoal(2) - obj.bnd.ymin)/(obj.bnd.ymax...
                    - obj.bnd.ymin)];

                qnorm(1) = obj.iCDF(rand(1),normgoal(1),obj.bias);
                qnorm(2) = obj.iCDF(rand(1),normgoal(2),obj.bias);
                qrand = [qnorm(1)*(obj.bnd.xmax - obj.bnd.xmin) + obj.bnd.xmin;...
                    qnorm(2)*(obj.bnd.ymax - obj.bnd.ymin) + obj.bnd.ymin];
                if (~obj.obsfield.isinside(qrand))
                    free = 1;
                end
            end
            %             if(obj.animate)
            %                 hold on
            %                 plot(qrand(1),qrand(2),'.',...
            %                     'color',[0 1 0],'MarkerSize',20);
            %                 hold off
            %             end
        end
        % genrate random configuraiton qrand
        function qrand = genqrand(obj)
            free = 0;
            while(~free)
                qrand = [rand(1)*(obj.bnd.xmax - obj.bnd.xmin) + obj.bnd.xmin;...
                    rand(1)*(obj.bnd.ymax - obj.bnd.ymin) + obj.bnd.ymin];
                if (~obj.obsfield.isinside(qrand))
                    free = 1;
                end
            end
        end
        % Inverse CDF
        function Fi = iCDF(obj,u,xg,p)
            a = (1-p)*xg;
            b = (1-p)*xg + p;
            m = (1-xg)/(1 - b);
            if(u >= 0 && u <= a)
                Fi = 1/(1-p)*u;
            elseif(u > a && u <= b)
                Fi = xg;
            elseif(u > b && u <= 1)
                Fi = m*(u - b) + xg;
            else
                Fi = inf;
            end
        end
        function checkqrand(obj,n)
            hold on;
            for i = 1:n
                qrand = obj.genqrandbiased;
                plot(qrand(1),qrand(2),'.')
            end
            hold off;
        end
        % Nodes
        function addnode(obj, n, varargin)
            obj.V = obj.V.addobj(n);
            if(isempty(varargin))
                obj.V(end).Value.index = length(obj.V);
            else
                obj.V(end).Value.index = varargin{1};
            end
        end
        % Edges
        function addedge(obj,e)
            %             e = edge(n1,n2);
            obj.E = obj.E.addobj(e);
        end
        % Build RRT algorithm
        function buildRRT(obj,N)
            for i = 1:N
                qrand = obj.genqrandbiased;
                extendRRT(obj,qrand);
            end
        end
        % Build RRT to Goal
        function buildtogoal(obj)
            qrand = obj.genqrandbiased;
            qnew = extendRRT(obj,qrand);
            while(norm(obj.qgoal - qnew) > obj.goalradius)
                qrand = obj.genqrandbiased;
                qnew = extendRRT(obj,qrand);
            end
        end
        % First Grow tree and try connecting goal
        function growandconnect(obj, growattempt)
            obj.buildRRT(growattempt);
            disp('attempt complete')

            goalnode = node(obj.qgoal);
            %             while(collision ~=0)
            %                 i = i-1;
            %                 goalnode.pindex = obj.V(i).Value.pindex;
            %                 checkedge = edge(obj.V(i).Value,goalnode);
            %                 collision = checkedge.iscolliding(obj.obs);
            %
            %                 if(i == 1)
            %                     break;
            %                 end
            %             end
            collision = 1;
            success = 0;
            while(success ~= 1)
                [~,nodeindex] = obj.nearestnode(obj.qgoal);
                checkedge = edge(obj.V(nodeindex).Value,goalnode);
                collision = checkedge.iscolliding(obj.obsfield);
                if(collision ~= 0)
                    obj.buildRRT(1);
                else
                    success = 1;
                end
            end

            if(collision == 0)
                obj.addedge(checkedge);
                goalnode.pindex = nodeindex;
                obj.addnode(goalnode);
                %                 obj.findpath;
            end
        end
        % For mobile robot use this instead of growandconnect
        % N is number of attempt
        function shootandconnect(obj,N)
            for i = 1:N
                obj.shootbranch;
                %                 obj.shootbranchrand;
            end
            disp('attempt complete')
            goalnode = node(obj.qgoal);
            collision = 1;
            success = 0;
            while(success ~= 1)
                [~,nodeindex] = obj.nearestnode(obj.qgoal);
                checkedge = edge(obj.V(nodeindex).Value,goalnode);
                collision = checkedge.iscolliding(obj.obsfield);
                if(collision ~= 0)
                    obj.shootbranch;
                    %                     obj.shootbranchrand;
                else
                    success = 1;
                end
            end

            if(collision == 0)
                obj.addedge(checkedge);
                goalnode.pindex = nodeindex;
                obj.addnode(goalnode);
                obj.findpath;
            end
            obj.drawtree;
        end
        % move to goal
        function movetogoal(obj)
            goalnode = node(obj.qgoal);
            while(norm(obj.V(end).Value.pose - obj.qgoal) > obj.goalradius)
                obj.shootbranch;
            end

            lastedge = edge(obj.V(end).Value,goalnode);
            obj.addedge(lastedge);
            goalnode.pindex = obj.V(end).Value.index;
            obj.addnode(goalnode);
            obj.findpath;
            obj.drawtree;
        end
        % generated qrand by considering semicircle;
        function qrand = genqrandSC(obj,index)
            qnode = obj.V(index).Value.pose;
            qvec = obj.makecolumn(obj.qgoal) - obj.makecolumn(qnode);
            thetaS = atan2(qvec(2),qvec(1));
            %             thetarand = rand(1)*pi - pi/2;
            thetarand = rand(1)*deg2rad(obj.thetarange)...
                - deg2rad(obj.thetarange/2);
            qrand = qnode + obj.step_size*[cos(thetarand + thetaS)...
                ; sin(thetarand + thetaS)];
        end
        % Test above function
        function testgenqrandSC(obj)
            %             figure
            for i=1:1000
                q = obj.genqrandSC(obj.V(end).Value.index);
                hold on
                plot(q(1),q(2),'.')
                hold off
            end
            axis('equal')
            grid on
        end
        % Extend (1 branch) RRT algorithm extending latest node only
        % optional input 1: index of node from where to extend branch
        % optional input 2: next generation no child index
        function newnodeid = shootbranch(obj,varargin)
            if(isempty(varargin))
                nodeindex = obj.V(end).Value.index;
            else
                nodeindex = varargin{1};
                nochildindex = varargin{2};
            end
            %             qrand = obj.genqrandbiased;
            qrand = obj.genqrandSC(obj.V(nodeindex).Value.index);
            qnode = obj.V(nodeindex).Value.pose;
            if(~iscolumn(qnode))
                qnode = qnode';
            end
            if(~iscolumn(qrand))
                qrand = qrand';
            end

            %             qnew = qrand - qnear
            qnew = qnode + obj.step_size*(qrand - qnode)./norm(qrand - qnode);
            %             qnew = qnear + obj.step_size*(qrand - qnear);
            edgepoint = obj.discedge(qnode, qnew);
            collision = 0;
            if(obj.bnd.isinside(qnew))
                for i = 1:size(edgepoint,1)
                    collision = collision + obj.obsfield.isinside(...
                        [edgepoint(i,1),edgepoint(i,2)]);
                end
            else
                collision = 1;
            end
            %             disp(collision)
            if(collision == 0)
                newnode = node(qnew);
                newnode.pindex = nodeindex;
                newnode.qrand = qrand;
                if(isempty(varargin))
                    obj.addnode(newnode);
%                     disp('upper')
                else
%                     newnodeid = (obj.gennum*10 + nochildindex);
                    obj.addnode(newnode);
                    newnodeid = obj.V(end).Value.index;
                    obj.V(nodeindex).Value.cindex = ...
                        [obj.V(nodeindex).Value.cindex newnodeid];
%                     disp('lower')
                end
                newedge = edge(obj.V(nodeindex).Value, newnode);
                obj.addedge(newedge);
            end
            if(obj.animate ~= 0)
                figure(1)
                obj.drawtree;
                %                 drawnow;
            end
        end
        % Extend (3 branch) RRT algorithm extending latest node only
        function shoot3branch(obj)
            nochildindexes = obj.nochildnodes;
            obj.nochildnodes = [];
            obj.gennum = obj.gennum + 1;
            n = 3;
            nextindex = 1;
            for i = 1:length(nochildindexes)
                for j = 1:n
                    obj.nochildnodes = [obj.nochildnodes...
                        obj.shootbranch(nochildindexes(i),nextindex)];
                    nextindex = nextindex + 1;
                end
            end
        end
        % Extend RRT algorithm extending from random node
        function qnew = shootbranchrand(obj)
            nodeindex = randperm(length(obj.V),1);
            qrand = obj.genqrandSC(nodeindex);
            qnode = obj.V(nodeindex).Value.pose;
            nodeindex = length(obj.V);
            if(~iscolumn(qnode))
                qnode = qnode';
            end
            if(~iscolumn(qrand))
                qrand = qrand';
            end

            %             qnew = qrand - qnear
            qnew = qnode + obj.step_size*(qrand - qnode)./norm(qrand - qnode);
            %             qnew = qnear + obj.step_size*(qrand - qnear);
            edgepoint = obj.discedge(qnode, qnew);
            collision = 0;
            if(obj.bnd.isinside(qnew))
                for i = 1:size(edgepoint,1)
                    collision = collision + obj.obsfield.isinside(...
                        [edgepoint(i,1),edgepoint(i,2)]);
                end
            else
                collision = 1;
            end
            %             disp(collision)
            if(collision == 0)
                newnode = node(qnew);
                newnode.pindex = nodeindex;
                newnode.qrand = qrand;
                obj.addnode(newnode);
                newedge = edge(obj.V(nodeindex).Value, newnode);
                obj.addedge(newedge);
            end
            if(obj.animate ~= 0)
                figure(1)
                obj.drawtree;
                %                 drawnow;
            end
        end
        % Find path in tree
        function W = findpath(obj)
            index = obj.V(end).Value.index;
            W(1) = index;
            i = 2;
            while(index ~= 1)
                index = obj.V(index).Value.pindex;
                W(i) = index;
                pathedge = edge(obj.V(W(i-1)).Value,obj.V(W(i)).Value);
                obj.path = obj.path.addobj(pathedge);
                i = i + 1;
            end
            figure(1)
            hold on;
            for j = 1:(length(W)-1)
                plot([obj.V(W(j)).Value.pose(1),obj.V(W(j+1)).Value.pose(1)],...
                    [obj.V(W(j)).Value.pose(2),obj.V(W(j+1)).Value.pose(2)],...
                    "color",[0 0.8 0],"LineWidth",2.5);
            end
            f1 = gca;
            title(f1, sprintf('RRT, path length = %f, Nodes: %d',...
                obj.pathlength, obj.V(end).Value.index));
            hold off;
        end
        % Calculate path length
        function l = pathlength(obj)
            l = 0;
            for i = 1:length(obj.path)
                l = l + obj.path(i).Value.edgelength;
            end
        end
        % Find pose of nearest node in V to q
        function [pose,nodeindex] = nearestnode(obj,q)
            for i = 1:obj.V(end).Value.index
                distance(i) = obj.V(i).Value.distancefrom(q);
            end
            [dmin, imin] = min(distance);
            pose = obj.V(imin).Value.pose;
            nodeindex = imin;
        end
        % check if the edge is intersecting
        function collision = checkcollision(obj,q,qnew)
            if(~iscolumn(q))
                q = q';
            end
            if(~iscolumn(qnew))
                qnew = qnew';
            end
            collision = 0;
            edgepoint = obj.discedge(q, qnew);
            for i = 1:size(edgepoint,1)
                collision = collision + obj.obsfield.isinside(...
                    [edgepoint(i,1),edgepoint(i,2)]);
            end
            node1 = node(q);
            node2 = node(qnew);
            e = edge(node1,node2);
            for i = 1:(length(obj.E)-1)
                collision = collision + e.checkintersection(obj.E(i).Value);
            end
        end
        % Extend RRT algorithm
        function qnew = extendRRT(obj,qrand)
            [qnear,nodeindex] = obj.nearestnode(qrand);
            if(~iscolumn(qnear))
                qnear = qnear';
            end
            if(~iscolumn(qrand))
                qrand = qrand';
            end

            %             qnew = qrand - qnear
            qnew = qnear + obj.step_size*(qrand - qnear)./norm(qrand - qnear);
            %             qnew = qnear + obj.step_size*(qrand - qnear);
            edgepoint = obj.discedge(qnear, qnew);
            collision = 0;
            for i = 1:size(edgepoint,1)
                collision = collision + obj.obsfield.isinside(...
                    [edgepoint(i,1),edgepoint(i,2)]);
            end
            %             disp(collision)
            if(collision == 0 && obj.bnd.isinside(qnew))
                newnode = node(qnew);
                newnode.pindex = nodeindex;
                newnode.qrand = qrand;
                obj.addnode(newnode);
                newedge = edge(obj.V(nodeindex).Value, newnode);
                obj.addedge(newedge);
            end
            if(obj.animate ~= 0)
                figure(1)
                obj.drawtree;
            end
        end
        % discretize edge
        function Edge = discedge(obj,qnear,qnew)
            d = round(obj.bnd.xmax*norm(qnear - qnew));
            npoints = 20*d;
            if(npoints < 10)
                npoints = 10;
            end
            x = linspace(qnear(1),qnew(1),npoints);
            y = linspace(qnear(2),qnew(2),npoints);
            Edge = [x' y'];
        end
        % draw tree at once
        function drawtree(obj)
            % nodes
            %             plot(obj.V(1).Value.pose(1),obj.V(1).Value.pose(2),'.',...
            %                 'color',[1 0.6 0],'MarkerSize',30);
            delete(obj.treehandle);
            obj.deletenodelabels;
            obj.drawstart;
            obj.drawgoal;
            hold on
            for i = 1:length(obj.V)
                obj.treehandle(i,1) = plot(obj.V(i).Value.pose(1),obj.V(i).Value.pose(2),'.',...
                    'color',[0 0 0],'MarkerSize',8);
                if(obj.shownodeindices == 1)
                    obj.nodetexthndl(i).value = text(obj.V(i).Value.pose(1)...
                        ,obj.V(i).Value.pose(2),num2str(obj.V(i).Value.index));
                    %                 s = t.FontSize;
                    obj.nodetexthndl(i).value.FontSize = 8;
                end
            end
            % edges
            for i = 1:length(obj.E)
                obj.treehandle(i,2) = obj.E(i).Value.drawedge;
            end
            f1 = gca;
            title(f1, sprintf('RRT, path length = %f, Nodes: %d',...
                obj.pathlength, obj.V(end).Value.index));
            hold off
        end
        % delete node labels from figure
        function deletenodelabels(obj)
            for i = 1:length(obj.nodetexthndl)
                delete(obj.nodetexthndl(i).value);
            end
        end
        % draw goal
        function drawgoal(obj)
            hold on
            plot(obj.qgoal(1),obj.qgoal(2),'.',...
                'color',[1 0 1],'MarkerSize',30);
            hold off
        end
        % draw start
        function drawstart(obj)
            hold on
            w = 0.3;h = 0.3;
            rectangle('Position',[obj.V(1).Value.pose(1)-w/2,obj.V(1).Value.pose(2)-h/2,w,h],...
                'FaceColor',[1 1 0],'EdgeColor','black','LineWidth',0.5);
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
    end
end