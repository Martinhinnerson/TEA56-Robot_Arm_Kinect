function [p_new] = rotation_fix(p)

v = -2*pi/180;
n = length(p);
p_new = zeros(n,2);
for i = 1:n
    p_new(i,:) = [cos(v) -sin(v); sin(v) cos(v)]*p(i,:)';
end

end