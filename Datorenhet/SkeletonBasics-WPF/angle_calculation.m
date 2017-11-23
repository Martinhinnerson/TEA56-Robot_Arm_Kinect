function [angles] = angle_calculation(p)
n = length(p);
angles = zeros(n-1,1);

if(p(2,1) < 0)
    angles(1) = 180 + radtodeg(atan((p(2,2)-p(1,2))/(p(2,1)-p(1,1))));
else
    angles(1) = radtodeg(atan((p(2,2)-p(1,2))/(p(2,1)-p(1,1))));
end
b = 0.16;
a = 0.047;
c = 0.15;
v = asin(a/c);

help_pos = [b*cos(v+degtorad(angles(1))), b*sin(v+degtorad(angles(1)))];
l1 = p(2,:) - help_pos;

L4 = cross([l1, 0],[p(3,:) - p(2,:), 0]);
distance = norm(l1)*norm(p(3,:) - p(2,:));
angles(2) = radtodeg(acos(dot(l1,p(3,:)- p(2,:))/distance));
if (L4(3) < 0)
    angles(2) = -angles(2);
end

for i = 3:n-1
    distance = pdist([p(i,:);p(i-1,:)],'euclidean')*pdist([p(i+1,:);p(i,:)],'euclidean');
    angles(i) = radtodeg(acos(dot(p(i,:)- p(i-1,:),p(i+1,:)- p(i,:))/distance));
end
l1 = p(3,:)- p(2,:);
L4 = cross([l1, 0],[p(4,:) - p(3,:), 0]);
if (L4(3) < 0)
    angles(3) = -angles(3);
end

end
