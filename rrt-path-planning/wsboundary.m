% This class defines the workspace boundary
classdef wsboundary < handle
    properties
        xmin = 0;
        xmax = 10
        ymin = 0;
        ymax = 10;
    end
    methods
        function obj = wsboundary()

        end
        function in = isinside(obj,pq)
            if(~iscolumn(pq))
                pq = pq';
            end

            if(pq(1)>obj.xmin && pq(1)<obj.xmax...
                    && pq(2)>obj.ymin && pq(2)<obj.ymax)
                in = 1;
            else
                in = 0;
            end
        end
        function obj = set(obj,xmin,xmax,ymin,ymax)
            obj.xmin = xmin;
            obj.xmax = xmax;
            obj.ymin = ymin;
            obj.ymax = ymax;
        end
        function drawboundary(obj)
            hold on
            plot([obj.xmin,obj.xmax],[obj.ymin,obj.ymin],"color",[0 0 0],"LineWidth",2);
            plot([obj.xmin,obj.xmax],[obj.ymax,obj.ymax],"color",[0 0 0],"LineWidth",2);
            plot([obj.xmin,obj.xmin],[obj.ymin,obj.ymax],"color",[0 0 0],"LineWidth",2);
            plot([obj.xmax,obj.xmax],[obj.ymin,obj.ymax],"color",[0 0 0],"LineWidth",2);
            hold off
        end
    end
end