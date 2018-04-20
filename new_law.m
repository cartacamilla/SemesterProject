clc
clear
close all


DS1 = @(x) [-5 0; 0 -5] * x;
DS2 = @(x) [-5 0; 0 -5] * x;

Lim_ws = @(x) [min(max(x(1),-5),5) ; min(max(x(2),-5),5)];


% loop params
dt = 0.01;
t = 0;
T_final = 2;

% intial locaiton of the two objects
pos_red = [-1;-2];
pos_green = [1;-2];


% robot's param
pos_robot = [0;5];
vel_robot = [0;0];

D = 5;
M = 1;

b1 = 0.0;
b2 = 1 - b1;
B = [b1;b2];
epsilon = 5;

vel_red = [0;0];
vel_green = [0;0];


figure;
hold on
grid on
axis([-5 5 -5 5])

direction_red = pi/2;
direction_green = pi;

Error = [];
while(t < T_final)
    
    % desired velocity to reach each object
    f1 = DS1(pos_robot - pos_red);
    f2 = DS2(pos_robot - pos_green);
    

    if(norm(f1) > 4), f1 = f1 ./ norm(f1) * 4; end
        if(norm(f2) > 4), f2 = f2 ./ norm(f2) * 4; end


    % adaptations
    adapt_error_1 = vel_robot'*vel_red;
    adapt_error_2 = vel_robot'*vel_green;
    adapt_error = b1.*adapt_error_1 + b2 .* adapt_error_2;
    
    Error(end+1,:) = [adapt_error_1, adapt_error_2, adapt_error];

    term1 = vel_robot'*(vel_red - vel_green);
%     term2 = -0.5 * (b1 * vel_red + b2 * vel_green)' * (f1 + f2) ;
    term2 =  (b1 * vel_red + b2 * vel_green)' * (f1-f2) ;
    
    b1_dot = - epsilon * (term1 + term2);
    b1 = b1 + b1_dot .* dt;
    if(b1>1),b1=1;end
    if(b1<0),b1=0;end
    b2 = 1-b1;
    
    B(:,end+1) = [b1;b2];
    
    vel_desired = b1 .* f1 + b2 .* f2;
    
    
    % robot's movement
    F_control = -1 * D .* (vel_robot - vel_desired);
    acc = (1/M) * (F_control);
    vel_robot = vel_robot + acc   .* dt ;
    pos_robot   = pos_robot   + vel_robot .* dt;
    
    
    % objects movements
    
    direction_red = direction_red + 0.2 * randn;
    direction_green = direction_green + 0.2 * randn;
    
    vel_red = 0.2* [cos(direction_red) ; sin(direction_red)];
    vel_green = 0.2* [cos(direction_green) ; sin(direction_green)];
    
    pos_red = pos_red + vel_red;
    pos_green = pos_green + vel_green;
    
    if(b1 > 0.5)
        pos_red = pos_red + 0.05* (pos_robot - pos_red);        
    else
        pos_green = pos_green + 0.05* (pos_robot - pos_green);
        
    end
    
    
    
    % limiting movements to workspace
    pos_robot = Lim_ws(pos_robot);
    pos_red   = Lim_ws(pos_red);
    pos_green = Lim_ws(pos_green);
    
    
    % plotting
    cla
    plot(pos_robot(1),pos_robot(2), 'xk');
    plot(pos_red(1),pos_red(2), 'or');
    plot(pos_green(1),pos_green(2), 'og');
    
    text(pos_red(1)+0.2,pos_red(2), num2str(b1));
    text(pos_green(1)+0.2,pos_green(2), num2str(b2));

    
    quiver(pos_robot(1),pos_robot(2),vel_robot(1)*dt,vel_robot(2)*dt,20)
    
    
    
    drawnow
    pause(0.01)
    
    
    
    t = t + dt;
    
    
end

%%


figure;
hold on
plot(Error(:,1),'r','LineWidth',1.4)
plot(Error(:,2),'g','LineWidth',1.4)
plot(Error(:,3),'--k','LineWidth',1.4)

xlabel('Time [s]')
ylabel('Error (inner product)')
legend('Task 1', 'Task 2', 'Composite')
grid on






