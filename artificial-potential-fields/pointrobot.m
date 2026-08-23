classdef pointrobot < handle
    properties
        pose  % position of robot
        Ofield
        path   % [x ; y]
        alpha = 0.01/2;
        tol = 0.1;
        gradnorm;
        LMtol = 0.2;
    end
    methods
        function obj = pointrobot(ofield)
            obj.pose = ofield.makecolumn(ofield.qstart);
            obj.Ofield = ofield;
        end
        function obj = validqstart(obj,qstart)
            if(obj.obsfield.isinside(qstart))
                error('Start position is inside obstacle');
            end
            obj.qstart = qstart;
        end
        function normF = advancerobot(obj)
            Fatt = obj.Ofield.goalfield(obj.pose);
            Frep = obj.Ofield.repfield(obj.pose);
            F = Fatt + Frep;
            normF = norm(F);
            disp(normF);
            if(normF < obj.LMtol)
                error('Local minima');
            end
            obj.pose = obj.pose + obj.alpha*norm(F)*F;
            if (~obj.Ofield.bnd.isinside(obj.pose))
                error('Robot is outside the work boundary');
            end
            obj.drawpose;
        end
        % go to goal
        function obj = movetogoal(obj)
            obj.path(1,:) = obj.Ofield.makecolumn(obj.Ofield.qstart)';
            i = 2;
            while(norm(obj.pose - obj.Ofield.qgoal) > obj.tol)
                obj.gradnorm(i-1) = obj.advancerobot;
                obj.path(i,:) = obj.pose;
                i = i + 1;
            end
        end
        % update robot pose on figure
        function drawpose(obj)
            hold on
            plot(obj.pose(1),obj.pose(2),'.','color',[0 0 0],'MarkerSize',2);
            hold off
        end
        function drawpath(obj,color)
            hold on
            plot(obj.path(:,1),obj.path(:,2),color,'LineWidth',1.5);
            hold off
        end
    end
end