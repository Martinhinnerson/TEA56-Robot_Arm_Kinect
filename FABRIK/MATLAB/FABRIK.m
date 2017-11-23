function [ p_new ] = FABRIK(p,t,allowed_theta_f)
if(t(2) < 0 || norm(t) < 3)
    p_new = p;
    return;
end
n = length(p);
d = zeros(n - 1,1);
for i = 1:(n -1)
    X = p(i,:) - p(i+1,:);
    d(i) = norm(X);
end
%total_d = sum(d);
%plot_arm(p,t,total_d);

dist = norm(p(1,:) - t);


%Om armen är utanför räckvidden, kör bara en iteration av forward reaching
if dist > sum(d)
    for i = 1:n-1
        r = norm(t - p(i,:));
        gamma = d(i)/r;
        p(i+1,:) = (1 - gamma)*p(i,:) + gamma*t;
    end
else
    %plot_arm(p,t,total_d);
    b = p(1,:);
    last_dif = 30;
    dif = norm(p(n,:) - t);
    while(dif > 0.01 && last_dif - dif > 0.000001)
        %plot_arm(p,t,total_d);
        p(n,:) = t;
        %plot_arm(p,t,total_d);
        r = norm(p(n-1,:) - p(n,:));
        gamma = d(n-1)/r;
        p(n-1,:) = (1 - gamma)*p(n,:) + gamma*p(n-1,:);   
        %plot_arm(p,t,total_d);
        for i = 2:n-1
            i2 = n - i;
            %p(i2,:) = follow_the_rules(p(i2+1,:),p(i2+2,:),p(i2,:),allowed_theta_b(i2+1,:));
            r = norm(p(i2,:) - p(i2+1,:));
            gamma = d(i2)/r;
            p(i2,:) = (1 - gamma)*p(i2+1,:) + gamma*p(i2,:);    
            %plot_arm(p,t,total_d);
        end
               
        p(1,:) = b;
        %plot_arm(p,t,total_d);
        p(2,:) = n_rules(p(1,:),[0 -1],p(2,:),allowed_theta_f(1,:));
        r = norm(p(1,:) - p(2,:));
        gamma = d(1)/r;
        p(2,:) = (1 - gamma)*p(1,:) + gamma*p(2,:); 
%         plot_arm(p,t,total_d);
        for i = 2:n-1
            p(i+1,:) = n_rules(p(i,:),p(i-1,:),p(i+1,:), allowed_theta_f(i,:));
            r = norm(p(i,:) - p(i+1,:));
            gamma = d(i)/r;
            p(i+1,:) = (1 - gamma)*p(i,:) + gamma*p(i+1,:); 
%             plot_arm(p,t,total_d);
        end
        last_dif = dif;
        dif = norm(p(n,:) - t);
        %plot_arm(p,t,total_d);
    end
end

%plot_arm(p,t,total_d);
p_new = p;
end

