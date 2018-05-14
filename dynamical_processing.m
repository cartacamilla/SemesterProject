function [centroid, layer, robot, tdone] = dynamical_processing(centroid,layer,robot,iter, MAX_LAYER,DECAY_MAX,dt,vidObj)
%dynamical_processing Summary of this function goes here
%   Detailed explanation goes here
tstart = tic;


%Workspace limits
Lim_ws = @(x) [min(max(x(1),1),vidObj.Width); 
               min(max(x(2),1),vidObj.Height)];

%Robot's parameters
D = 5;
M = 1;
epsilon = 0.1;

[layer, centroid] = layer_handling(layer, centroid, iter, MAX_LAYER, DECAY_MAX);

layer.vel = layer.alpha_vel*layer.vel + ...
           (1-layer.alpha_vel)*(layer.pos(:,:,iter)-layer.pos(:,:,iter-1))/dt;

%identification velocity

[robot,layer] = adaptation_function(robot,layer,epsilon,dt,iter);

centroid.old_nbr = centroid.nbr;

% robot's movement
F_control = -1 * D .* (robot.vel-robot.vel_desired);
robot.acc = (1/M) * (F_control);
robot.vel = robot.vel + robot.acc.*dt;
robot.pos   = robot.pos   + robot.vel.*dt;

% limiting movements to workspace
robot.pos = Lim_ws(robot.pos);

tdone = toc(tstart);

    

plot(centroid.pos(1,:),centroid.pos(2,:),'*k','MarkerSize', 30,'LineWidth', 2);
    hold on;

plot(layer.pos(1,1,iter),layer.pos(2,1,iter),'or','MarkerSize', 30,...
     'LineWidth', 2);
hold on;
plot(layer.pos(1,2,iter),layer.pos(2,2,iter),'og','MarkerSize', 30,...
     'LineWidth', 2);
hold on;

[b_max,w] = max(robot.B);
if(b_max > 0.5)
    plot(layer.pos(1,w,iter),layer.pos(2,w,iter),'ow','MarkerSize',30,'LineWidth', 2);
    hold on;
end

quiver(robot.pos(1,:),robot.pos(2,:),robot.vel(1,:),robot.vel(2,:),'r')
hold on;
quiver(layer.pos(1,1,iter),layer.pos(2,1,iter), layer.vel(1,1),...
       layer.vel(2,1),5,'r')
hold on;
if(layer.nbr>1)
    quiver(layer.pos(1,2,iter),layer.pos(2,2,iter), layer.vel(1,2),...
           layer.vel(2,2),5,'r')
end
hold off;

    % desired velocity to reach each object
%     robot.F(:,1) = DS1(robot.pos - layer.pos(:,1,iter));
%     layer_pos_1 = layer.pos(:,1,iter);
%     if(dt)
%         layer.vel(:,1) = layer.alpha_vel*layer.vel(:,1) + ...
%                         (1-layer.alpha_vel)*(layer_pos_1 - layer.pos(:,1,iter-1))/dt;
%     else
%         layer.vel(:,1) = [0;0];
%     end

%     robot.F(:,1) = robot.F(:,1)  + layer.vel(:,1);
% 
%     if(layer.nbr > 1)
%         robot.F(:,2) = DS2(robot.pos - layer.pos(:,2,iter));
%         layer_pos_2 = layer.pos(:,2,iter);                      
%         layer.vel(:,2) = layer.alpha_vel*layer.vel(:,2) + ...
%                         (1-layer.alpha_vel)*(layer_pos_2 - layer.pos(:,2,iter-1))/dt;
%         robot.F(:,2) = robot.F(:,2) + layer.vel(:,2);
%         if(robot.B(1) == 1)
%             plot(layer.pos(1,1,iter),layer.pos(2,1,iter),'ow','MarkerSize',...
%                  30,'LineWidth', 2);
%             hold on
%         else 
%             plot(layer.pos(1,2,iter),layer.pos(2,2,iter),'ow','MarkerSize',...
%                  30,'LineWidth', 2);
%             hold on
%         end
%     else
%         robot.F(:,2) = [0;0]; 
%         layer.vel(:,2) = [0;0]; 
%     end
    
    
    %adaptations
    
    
%     p1 = robot.vel'*layer.vel(:,1);
%     p2 = robot.vel'*layer.vel(:,2);
%     
%     p1_plus = max(0,p1);
%     p1_minus = min(0,p1);
%     
%     p2_plus = max(0,p2);
%     p2_minus = min(0,p2);
%     
%     adapt_error_1 = p1_minus;
%     adapt_error_2 = p2_minus;
%     adapt_error_3 = max(-robot.B(1)*p1_plus, -robot.B(2)*p2_plus);
%     adapt_error = robot.B(1).*adapt_error_1 + robot.B(2).*adapt_error_2 + ...
%                   robot.B(3).*adapt_error_3;
%     pos_err_1 = norm(robot.pos - layer.pos(:,1,iter));
%     pos_err_2 = norm(robot.pos - layer.pos(:,2,iter));
%     adapt_error_1 = (robot.vel'*layer.vel(:,1) + pos_err_1);
%     adapt_error_2 = (robot.vel'*layer.vel(:,2) + pos_err_2);
%     adapt_error = robot.B(1).*adapt_error_1 + robot.B(2).*adapt_error_2;

%     robot.Error(end+1,:) = [adapt_error_1, adapt_error_2,adapt_error_3,...
%                             adapt_error];
%     
%     term1 = robot.vel'*(layer.vel(:,1) - layer.vel(:,2));
%     term2 =  (robot.B(1)*layer.vel(:,1) + robot.B(2)*layer.vel(:,2))'*...
%              (robot.F(:,1) - robot.F(:,2)) ;
% 
% %     b1_dot = - epsilon * (term1 + term2 + term3);
%     b1_dot = - epsilon * (term1 + term2)
%     robot.B(1) = robot.B(1) + b1_dot .* dt;
%     robot.b1_dot(end+1) = b1_dot;
%     if(robot.B(1)>1),robot.B(1)=1;end
%     if(robot.B(1)<0),robot.B(1)=0;end

    %should take into account the case with only one centroid
%     if(layer.nbr==1)
%         robot.B(1)=1;
%     end
%     robot.B(2) = 1-robot.B(1);
%     robot.B_log(:,end+1) = [robot.B(1); robot.B(2)];
%     
% %     robot.vel_desired = robot.B(1).*robot.F(:,1) + robot.B(2).*robot.F(:,2);
%     robot.vel_desired = sum(robot.B.*robot.F , 2);

    
end

