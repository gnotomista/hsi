function [distance, u, v] = minimumDistance(p, a, b)

[distance, u, v] = SqDistPointSegment(p,a,b);

end

function d = distancePtToPt(p1,p2)
d = norm(p2 - p1,2);
end

function [d,u,v] = SqDistPointSegment(p,a,b)
[pt,u,v] = ClosestPtPointSegment(p,a,b);
d = distancePtToPt(p,pt);
end

function [d,u,v] = ClosestPtPointSegment(c,a,b)
ab = b-a;
t = (c-a)*ab'/(ab*ab');
if t < 0
    t = 0;
elseif t > 1
    t = 1;
end
d = a + t * ab;
u=t;
v=1-u;
end