function [new_pos] = n_rules(p,p2,t,theta)
new_pos = t;
L1 = p - p2;
L2 = t - p;
help_angle = acos(dot(L2,L1)/(norm(L2)*norm(L1)));

% O =  ProjectPoint([p;p2],t)';
% S = pdist([p;O],'euclidean');
% L3 = t - O;
L4 = cross([L1, 0],[L2, 0]);

if(L4(3) > 0)
    v = theta(1);
elseif (L4(3) < 0)
    v = -theta(2);
else
    new_pos = p + L1;
    return;
end

if(help_angle < abs(v))
    return
else
    L5 = [cos(v) -sin(v); sin(v) cos(v)]*L1';
    end1 = (L5'+p);
    end2 = (L1 + t);
    start1 = p;
    start2 = t;
    [new_pos(1), new_pos(2)] = my_intersection([start1(1) start2(1); end1(1) end2(1)],[start1(2) start2(2);end1(2) end2(2)]);
end
end