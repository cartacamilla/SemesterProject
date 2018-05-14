function b_dot = winner_takes_all(b_dot_tilde, robot, nbr_tasks) 
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
% b_dot = zeros(1,nbr_tasks);

[~,I] = sort(b_dot_tilde,'descend');
w = I(1); 

if robot.B(w) == 1
    b_dot = zeros(1,nbr_tasks);
    return
else
    n = I(2);
    z = (b_dot_tilde(w) + b_dot_tilde(n))/2;
    b_dot = b_dot_tilde - z;
end

dot_sum = 0;

for i = 1:nbr_tasks
    if (robot.B(i) ~= 0 || b_dot(i) > 0)
        dot_sum = dot_sum + b_dot(i);
    end
end
b_dot(w) = b_dot(w) - dot_sum;

end
