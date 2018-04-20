function [layer, update_index] = backward_search(layer,centroid,iter)
%forward_search Summary of this function goes here
%   Detailed explanation goes here
dist = NaN(centroid.nbr,layer.nbr);
update_index = zeros(1,centroid.nbr);
for i = 1:centroid.nbr
    dist(i,:) = sqrt(sum((layer.pos(:,:,iter-1)-centroid.pos(:,i)).^2));
    [~,update_index(i)] = min(dist(i,:));   
    layer.pos(:,update_index(i),iter) = ...
        layer.a_filt*layer.pos(:,update_index(i),iter-1)...
        +(1-layer.a_filt)*centroid.pos(:,i);
    layer.decay(update_index(i)) = 0;
end

end

