classdef Point
    properties
        x
        y
    end
    methods
        function obj = Point(x,y)
            obj.x = x;
            obj.y = y;
        end
        % Given three collinear points p, q, r, the function checks if
        % point q lies on line segment 'pr'
        function on = onSegment(p,q,r)
            if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x)...
                    && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
                on = 1;
            else
                on = 0;
            end
        end
        % To find orientation of ordered triplet (p, q, r).
        % The function returns following values
        % 0 --> p, q and r are collinear
        % 1 --> Clockwise
        % 2 --> Counterclockwise
        function or = orientation(p, q, r)
            val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

            if (val == 0)
                or = 0;
            elseif (val > 0) % collinear
                or = 1;
            else
                or = 2; % clock or counterclock wise
            end
        end
        % The main function that returns true if line segment 'p1q1'
        % and 'p2q2' intersect.
        function intersect = doIntersect(p1, q1, p2, q2)


            % Find the four orientations needed for general and
            % special cases
            o1 = orientation(p1, q1, p2);
            o2 = orientation(p1, q1, q2);
            o3 = orientation(p2, q2, p1);
            o4 = orientation(p2, q2, q1);

            % General case
            if (o1 ~= o2 && o3 ~= o4)
                intersect = 1;

                % Special Cases
                % p1, q1 and p2 are collinear and p2 lies on segment p1q1
            elseif (o1 == 0 && onSegment(p1, p2, q1))
                intersect = 1;

                % p1, q1 and q2 are collinear and q2 lies on segment p1q1
            elseif (o2 == 0 && onSegment(p1, q2, q1))
                intersect = 1;

                % p2, q2 and p1 are collinear and p1 lies on segment p2q2
            elseif (o3 == 0 && onSegment(p2, p1, q2))
                intersect = 1;

                % p2, q2 and q1 are collinear and q1 lies on segment p2q2
            elseif (o4 == 0 && onSegment(p2, q1, q2))
                intersect = 1;

            else
                intersect = 0; % Doesn't fall in any of the above cases
            end

        end
    end
end