function [layer,centroid] = layer_handling(layer,centroid,iter, MAX_LAYER, DECAY_MAX)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
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
            layer.pos(:,update_index,iter) = layer.alpha_pos*layer.pos(:,update_index,iter-1)...
                +(1-layer.alpha_pos)*centroid.pos(:,i);
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
                                +layer.v_norm*robot_dist/norm(robot_dist);
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
            layer.pos(:,update_index,iter) = layer.alpha_pos*layer.pos(:,update_index,iter-1)...
                +(1-layer.alpha_pos)*centroid.pos(:,i);
            layer.decay(update_index) = 0;
        end  


    %handles case where centroids disappear
    elseif(centroid.old_nbr > centroid.nbr)
        for i = 1:centroid.nbr
            %closest layer to centroid gets it (add prec position) 
            dist = sqrt(sum((layer.pos(:,:,iter-1)-centroid.pos(:,i)).^2));
            [~,not_ghost(i)] = min(dist);   
            layer.pos(:,not_ghost(i),iter) = layer.alpha_pos*layer.pos(:,not_ghost(i),iter-1)...
                                             +(1-layer.alpha_pos)*centroid.pos(:,i);
            layer.decay(not_ghost(i)) = 0;
        end
        ghost_l_index = setdiff(layer_index,not_ghost);
        layer.pos(:,ghost_l_index,iter) = layer.pos(:,ghost_l_index,iter-1);
    end
end

