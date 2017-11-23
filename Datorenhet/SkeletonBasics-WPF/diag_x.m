function [] = diag_x(x_real,y_real,x_array,y_array)
n = length(x_real);
t = 0:n-1;
subplot(2,2,1)
plot(t,x_real);
title('Indata X');
axis([0 n 0 0.6]);
subplot(2,2,2)
plot(t,y_real);
title('Indata Y');
axis([0 n -0.2 0.6]);
subplot(2,2,3)
plot(t,x_array);
title('Justerad X');
axis([0 n 0 0.6]);
subplot(2,2,4)
plot(t,y_array);
title('Justerad Y');
axis([0 n -0.2 0.6]);
end