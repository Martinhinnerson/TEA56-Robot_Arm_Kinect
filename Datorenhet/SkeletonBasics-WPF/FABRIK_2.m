function [ p_new, theta_new ] = FABRIK_2(p_column,t)
allowed_theta_f = [2*pi/5 pi/2; pi/3 3*pi/4; pi/2 pi/2; 0 0];
t = t - [0.1 0];
p = vec2mat(p_column',2);
iter = 0;
n = length(p);
d = zeros(n - 1,1);
for i = 1:(n -1)
    X = p(i,:) - p(i+1,:);
    d(i) = norm(X);
end
total_d = sum(d);

if(norm(t) < 0.10 || ((abs(t(1)) < 0.12) && t(2) < 0) ||  t(2) < -0.14)
    p = [p; p(n,:) + [0.1 0]];
    p_new = reshape(p',8,1);
    angles = angle_calculation(p);
    theta_new = reshape(angles,3,1);
    return;
end

%plot_arm(p,t,total_d,iter);
dist = norm(p(1,:) - t);
%Om armen är utanför räckvidden, kör bara en iteration av forward reaching
if dist > sum(d)
    for i = 1:n-1
        t2 = t;
        t2 = more_rules(p(i,:),t2);
        r = norm(t2 - p(i,:));
        gamma = d(i)/r;
        p(i+1,:) = (1 - gamma)*p(i,:) + gamma*t2;
    end
    iter = iter + 1;
else
    %plot_arm(p,t,total_d,iter);
    b = p(1,:);
    last_dif = 30;
    dif = norm(p(n,:) - t);
    while(dif > 0.01 && iter < 50)
        if(last_dif - dif > 0.0001)
            p = rotation_fix(p);
        end
        %plot_arm(p,t,total_d,iter);
        p(n,:) = t;
        %plot_arm(p,t,total_d,iter);
        %p(n-1,:) = more_rules(p(n,:),p(n-1,:));
        r = norm(p(n-1,:) - p(n,:));
        gamma = d(n-1)/r;
        p(n-1,:) = (1 - gamma)*p(n,:) + gamma*p(n-1,:);   
        %plot_arm(p,t,total_d,iter);
        for i = 2:n-1
            i2 = n - i;
            p(i2,:) = more_rules(p(i2+1,:),p(i2,:));
            %p(i2,:) = follow_the_rules(p(i2+1,:),p(i2+2,:),p(i2,:),allowed_theta_b(i2+1,:));
            r = norm(p(i2,:) - p(i2+1,:));
            gamma = d(i2)/r;
            p(i2,:) = (1 - gamma)*p(i2+1,:) + gamma*p(i2,:);    
            %plot_arm(p,t,total_d,iter);
        end
               
        p(1,:) = b;
        %plot_arm(p,t,total_d,iter);
        p(2,:) = n_rules(p(1,:),[0 -1],p(2,:),allowed_theta_f(1,:));
        p(2,:) = more_rules(p(1,:),p(2,:));
        r = norm(p(1,:) - p(2,:));
        gamma = d(1)/r;
        p(2,:) = (1 - gamma)*p(1,:) + gamma*p(2,:); 
%         %plot_arm(p,t,total_d,iter);;
        for i = 2:n-1
            p(i+1,:) = n_rules(p(i,:),p(i-1,:),p(i+1,:), allowed_theta_f(i,:));
            p(i+1,:) = more_rules(p(i,:),p(i+1,:));
            r = norm(p(i,:) - p(i+1,:));
            gamma = d(i)/r;
            p(i+1,:) = (1 - gamma)*p(i,:) + gamma*p(i+1,:); 
%             %plot_arm(p,t,total_d,iter);;
        end
        last_dif = dif;
        dif = norm(p(n,:) - t);
        iter = iter + 1;
    end
end
p = [p; p(n,:) + [0.1 0]];
%plot_arm(p,t,total_d,iter);
p_new = reshape(p',8,1);
angles = angle_calculation(p);
theta_new = reshape(angles,3,1);
end

