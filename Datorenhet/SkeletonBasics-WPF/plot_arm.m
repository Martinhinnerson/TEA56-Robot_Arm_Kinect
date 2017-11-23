function [] = plot_arm(p,t,d,iterations)
t = t + [0.1 0];
n = length(p);
c = linspace(1,10,n+2);
figure(1);
d =0.6;
angles = angle_calculation(p);
b = 0.143;
a = 0.046;
v = atan(a/b);

help_pos = [b*cos(v+degtorad(angles(1))), b*sin(v+degtorad(angles(1)))];
scatter([p(:,1); t(1,1);help_pos(1,1)],[p(:,2);t(1,2);help_pos(1,2)],[],c);
axis([-0.2 0.6 -0.2 0.6]);

for i = 1:n-1
line([p(i,1) p(i+1,1)],[p(i,2) p(i+1,2)]);
end

if(p(n,:) ~= t)
diff_t = ['diff =', num2str(sprintf('%.2f',pdist([p(n,:);t],'euclidean')))];
text(0.8*d,0.9*d,diff_t);
end

angles_t1 = ['theta_1 = ', num2str(round(angles(1)))];
angles_t2 = ['theta_2 = ', num2str(round(angles(2)))];
angles_t3 = ['theta_3 = ', num2str(round(angles(3)))];
%angles_t4 = ['theta_4 = ', num2str(round(radtodeg(angles(4))))];
text(0.8*d,0.8*d,angles_t1);
text(0.8*d,0.7*d,angles_t2);
text(0.8*d,0.6*d,angles_t3);
text(0.8*d,0.5*d,['Iter =', num2str(iterations)]);
%text(-0.9*d,0.5*d,angles_t4);

rectangle('Position',[-0.12 -0.2 0.24 0.20]);
line([-1 1],[-0.14 -0.14]);
end