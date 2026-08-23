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
        % method to be used with Obstacle class objects array
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
    end
end