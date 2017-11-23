function [p_new] = more_rules(p,p2)
if( p2(2) > 0 && p(2) > 0)
    p_new = p2;
    return
end
if(p(1) == p2(1))
p2(1) = p2(1)-0.01;
elseif(p(2) == p2(2))
p2(2) = p2(2) - 0.01;
end


help_pos1 = [30 30];
help_pos2 = [30 30];

if(p(2) == p2(2))
    [help_pos1(1), help_pos1(2),a] = my_intersection([p2(1), 0;p(1), 0.12],[p2(2), 0;p(2), 0]);
elseif(p(1) == p2(1))
    [help_pos2(1), help_pos2(2),b] = my_intersection([p2(1), 0.12;p(1),0.12],[p2(2), 0;p(2), -0.17]);
else
    [help_pos1(1), help_pos1(2),a] = my_intersection([p2(1), 0;p(1), 0.12],[p2(2), 0;p(2), 0]);
    [help_pos2(1), help_pos2(2),b] = my_intersection([p2(1), 0.12;p(1), 0.12],[p2(2), 0;p(2), -0.17]);
end

if((help_pos1(1) > 0 && help_pos1(1) < 0.12 && a)||( help_pos2(2) < 0 && help_pos2(2) > -0.17 && b))
    p_new =[0.12 0];
    if((p(1) > p2(1)) && ( help_pos2(2) < 0 && help_pos2(2) > -0.17 && b))
    p_new = help_pos2;
    end
else
    p_new = p2;
end

end