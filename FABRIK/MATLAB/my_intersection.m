function [xi, yi] =  my_intersection(x,y)
dx = diff(x);  %# Take the differences down each column
dy = diff(y);
den = dx(1)*dy(2)-dy(1)*dx(2);  %# Precompute the denominator
if(den == 0)
    error('Parallella linjer');
end
ua = (dx(2)*(y(1)-y(3))-dy(2)*(x(1)-x(3)))/den;
ub = (dx(1)*(y(1)-y(3))-dy(1)*(x(1)-x(3)))/den;

xi = x(1)+ua*dx(1);
yi = y(1)+ua*dy(1);
end