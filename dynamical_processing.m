function [centroid, layer, robot, tdone] = dynamical_processing(centroid,layer,robot,iter, MAX_LAYER,DECAY_MAX,dt,vidObj)
%dynamical_processing Summary of this function goes here
%   Detailed explanation goes here
tstart = tic;

p = 0.3;
a = 0.3;
v = 2;

%Dynamical systems
DS1 = @(x)  0.1*[-10 0; 0 -10] * x;
DS2 = @(x) 0.1*[-10 0; 0 -10] * x;

%Workspace limits
Lim_ws = @(x) [min(max(x(1),1),vidObj.Width); 
               min(max(x(2),1),vidObj.Height)];

%Robot's parameters
D = 5;
M = 1;
epsilon = 0.01;

plot(centroid.pos(1,:),centroid.pos(2,:),'*k','MarkerSize', 30,'LineWidth', 2);
    hold on;

    %unused layer decay
    if layer.nbr > 0
        for l = 1:layer.nbr
            layer.decay(l) = layer.decay(l) + 1;
        end
    end

    not_ghost = zeros(centroid.nbr,1);
    layer_index = linspace(1,centroid.old_nbr,centroid.old_nbr);


    %update layer if number of centroids is the same 
    if(centroid.old_nbr == centroid.nbr)
        %update known layers
        for i = 1:centroid.nbr
            %closest layer to centroid gets it (add prec position) 
            dist = sqrt(sum((layer.pos(:,:,iter-1)-centroid.pos(:,i)).^2));
            [~,update_index] = min(dist);   
            layer.pos(:,update_index,iter) = a*layer.pos(:,update_index,iter-1)...
                +(1-a)*centroid.pos(:,i);
            layer.decay(update_index) = 0;
        end   
        if(centroid.nbr < layer.nbr) 
            for j = 1:layer.nbr
                if(isnan(layer.pos(:,j,iter)))
                    layer.pos(:,j,iter) = layer.pos(:,j,iter-1);
                    if(layer.decay(j) > DECAY_MAX)
                        if(layer.out_ws(j)==false)
                            robot_dist = (layer.pos(:,j,iter-1)-robot.pos);
                            layer.pos(:,j,iter) = layer.pos(:,j,iter-1)...
                                +v*robot_dist/norm(robot_dist);
                        else
                            layer.pos(:,j,iter) = layer.pos(:,j,iter);
                        end
                        %Layer is out of ws
                        if(sum(layer.pos(:,j,iter) > Lim_ws(layer.pos(:,j,iter))))
                            layer.pos(:,j,iter) = layer.pos(:,j,iter);
                            layer.out_ws(j) = true;
                        end
                    end
                end
            end
        end

    %handle case where centroids appears
    elseif(centroid.old_nbr < centroid.nbr)
        for i = 1:centroid.nbr
            if(sum(isnan(layer.pos(:,i,iter-1))) && i <= MAX_LAYER)
                %initialize first layer
                if(layer.nbr == 0)
                    layer.pos(:,i,iter-1)=centroid.pos(:,i);
                    layer.nbr = i;
                elseif(layer.nbr > 0)
                    d = [];
                    for j = 1:centroid.nbr
                        d = [d; sqrt(sum((layer.pos(:,:,iter-1)-centroid.pos(:,j)).^2))];
                    end
                    [~,id] = max(d);
                    layer.pos(:,i,iter-1) = centroid.pos(:,id(1));
                end
            elseif(i > MAX_LAYER)
                disp('WHUT: i > MAXLAYER')
            end
        end
        for i = 1:centroid.nbr            
            layer.nbr = i;
            %closest layer to centroid gets it (add prec position) 
            dist = sqrt(sum((layer.pos(:,:,iter-1)-centroid.pos(:,i)).^2));
            [~,update_index] = min(dist);   
            layer.pos(:,update_index,iter) = a*layer.pos(:,update_index,iter-1)...
                +(1-a)*centroid.pos(:,i);
            layer.decay(update_index) = 0;
        end  


    %handles case where centroids disappear
    elseif(centroid.old_nbr > centroid.nbr)
        for i = 1:centroid.nbr
            %closest layer to centroid gets it (add prec position) 
            dist = sqrt(sum((layer.pos(:,:,iter-1)-centroid.pos(:,i)).^2));
            [~,not_ghost(i)] = min(dist);   
            layer.pos(:,not_ghost(i),iter) = a*layer.pos(:,not_ghost(i),iter-1)...
                                             +(1-a)*centroid.pos(:,i);
            layer.decay(not_ghost(i)) = 0;
        end
        ghost_l_index = setdiff(layer_index,not_ghost);
        layer.pos(:,ghost_l_index,iter) = layer.pos(:,ghost_l_index,iter-1);
    end

    plot(layer.pos(1,1,iter),layer.pos(2,1,iter),'or','MarkerSize', 30,'LineWidth', 2);
    hold on;
    plot(layer.pos(1,2,iter),layer.pos(2,2,iter),'og','MarkerSize', 30,'LineWidth', 2);
    hold on;
    centroid.old_nbr = centroid.nbr;




    % desired velocity to reach each object
    robot.F(:,1) = DS1(robot.pos - layer.pos(:,1,iter));
    layer_pos_1 = layer.pos(:,1,iter);
    if(dt)
        layer.vel(:,1) = p*layer.vel(:,1) + (1-p)*(layer_pos_1 - layer.pos(:,1,iter-1))/dt;
    else
        layer.vel(:,1) = [0;0];
    end

    robot.F(:,1) = robot.F(:,1)  + layer.vel(:,1);

    if(layer.nbr > 1)
        robot.F(:,2) = DS2(robot.pos - layer.pos(:,2,iter));
        layer_pos_2 = layer.pos(:,2,iter);                      
        layer.vel(:,2) = p*layer.vel(:,2) + (1-p)*(layer_pos_2 - layer.pos(:,2,iter-1))/dt;
        robot.F(:,2) = robot.F(:,2) + layer.vel(:,2);
        if(robot.B(1) == 1)
            plot(layer.pos(1,1,iter),layer.pos(2,1,iter),'ow','MarkerSize', 30,'LineWidth', 2);
            hold on
        else 
            plot(layer.pos(1,2,iter),layer.pos(2,2,iter),'ow','MarkerSize', 30,'LineWidth', 2);
            hold on
        end
    else
        robot.F(:,2) = [0;0]; 
        layer.vel(:,2) = [0;0]; 
    end
    
    
    %adaptations
    adapt_error_1 = robot.vel'*layer.vel(:,1);
    adapt_error_2 = robot.vel'*layer.vel(:,2);
    adapt_error = robot.B(1).*adapt_error_1 + robot.B(2).*adapt_error_2;
    
    robot.Error(end+1,:) = [adapt_error_1, adapt_error_2, adapt_error];
    
    term1 = robot.vel'*(layer.vel(:,1) - layer.vel(:,2));
    term2 =  (robot.B(1)*layer.vel(:,1) + robot.B(2)*layer.vel(:,2))'*...
             (robot.F(:,1) - robot.F(:,2)) ;

    b1_dot = - epsilon * (term1 + term2);
    robot.B(1) = robot.B(1) + b1_dot .* dt;
    robot.b1_dot(end+1) = b1_dot;
    if(robot.B(1)>1),robot.B(1)=1;end
    if(robot.B(1)<0),robot.B(1)=0;end

    %should take into account the case with only one centroid
%     if(layer.nbr==1)
%         robot.B(1)=1;
%     end
    robot.B(2) = 1-robot.B(1);
    robot.B_log(:,end+1) = [robot.B(1); robot.B(2)];
    
    robot.vel_desired = robot.B(1).*robot.F(:,1) + robot.B(2).*robot.F(:,2);

    % robot's movement
    F_control = -1 * D .* (robot.vel-robot.vel_desired);
    robot.acc = (1/M) * (F_control);
    robot.vel = robot.vel + robot.acc.*dt;
    robot.pos   = robot.pos   + robot.vel.*dt;

    % limiting movements to workspace
    robot.pos = Lim_ws(robot.pos);

tdone = toc(tstart);
    figure(1)
    quiver(robot.pos(1,:),robot.pos(2,:),robot.vel(1,:),robot.vel(2,:),'r')
    hold on
    quiver(layer.pos(1,1,iter),layer.pos(2,1,iter), layer.vel(1,1), layer.vel(2,1),5,'r')
    hold on
    if(layer.nbr>1)
        quiver(layer.pos(1,2,iter),layer.pos(2,2,iter), layer.vel(1,2), layer.vel(2,2),5,'r')
    end
    hold off
end

