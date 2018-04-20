function [layer, update_index] = forward_search(layer,centroid,iter)
%forward_search Summary of this function goes here
%   Detailed explanation goes here
dist = NaN(layer.nbr,centroid.nbr);
update_index = zeros(1,layer.nbr);
for i = 1:layer.nbr
    dist(i,:) = sqrt(sum((layer.pos(:,i,iter-1)-centroid.pos(:,:)).^2));
    [~,update_index(i)] = min(dist(i,:));   
    layer.pos(:,update_index(i),iter) = ...
        layer.a_filt*layer.pos(:,update_index(i),iter-1)...
        +(1-layer.a_filt)*centroid.pos(:,i);
    layer.decay(update_index(i)) = 0;
end

end

