classdef node
    properties
        pose        % position of the node
        index       % id of the node
        pindex      % id of parent node
        cindex      % ids of child nodes
        qrand       % sample used for generating this node
    end
    methods
        % Constructor
        function obj = node(pose)
            %            obj.dist = dist;
            obj = setpose(obj,pose);

        end
        function obj = setpose(obj,pose)
            if(~iscolumn(pose))
                pose = pose';
            end
            obj.pose = pose;
        end
        % Euclidean distance of point p from this node
        function dist = distancefrom(obj,p)
            if(~iscolumn(p))
                p = p';
            end
            dist = norm(obj.pose - p);
        end
        function obj = shootnode(obj,prevnode)
            thetarand = rand(1)*2*pi;
            pose = prevnode.pose + [obj.dist*cos(thetarand); obj.dist*sin(thetarand)];
            obj = obj.setpose(pose);
            %             disp(pose)
        end
    end
end