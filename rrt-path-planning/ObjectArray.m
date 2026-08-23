classdef ObjectArray
    properties
        Value
    end
    methods
        function obj = ObjectArray(F)
            if nargin ~= 0
                m = length(F);
                for i = 1:m
                    obj(i).Value = F(i);
                end
            end
        end
        function obj = addobj(obj,f)
            obj(length(obj)+1).Value = f;
        end
        % method to be used with Obstacle class object array
        function in = isinside(obj,pq)
            count = 0;
            for i = 1:length(obj)
                count = count + obj(i).Value.isinside(pq);
            end
            if(count ~= 0)
                in = 1;
            else
                in = 0;
            end
        end
        % method to be used with edge class object array
        % check if given edge is intersecting with atleast 1 edge in obj
        function intersect = checkintersection(obj,e)
            count = 0;
            for i = 1:length(obj)
                count = count + obj(i).Value.checkintersection(e);
            end
            if(count ~= 0)
                intersect = 1;
            else
                intersect = 0;
            end
        end
    end
end