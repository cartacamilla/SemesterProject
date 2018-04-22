function display_data(robot,centroid,layer,iter,time)
%display_data Summary of this function goes here
%   Detailed explanation goes here
clc
    disp('Robot position:')
    disp(robot.pos)
    disp('Robot velocity:')
    disp(robot.vel)
    disp('centroids positions:')
    disp(centroid.pos)
    disp('layer positions:')
    disp(layer.pos(:,:,iter))
    disp('Layers velocities:')
    disp(layer.vel)
    disp(['f1:',num2str(robot.F(1,1)),'  ',num2str(robot.F(2,1))])
    disp(['f2:',num2str(robot.F(1,2)),'  ',num2str(robot.F(2,2))])
    disp(['b1 = ' , num2str(robot.B(1))]);
    disp(['b2 = ' , num2str(robot.B(2))]);

    disp('Runtimes:')
    disp(['Outer dynamic= ',num2str(time.dynamic_out)])
    disp(['Percentage dynamic= ',num2str(time.dynamic_in/time.dynamic_out*100)])
    disp(['image_process= ',num2str(time.im_proc)])
    
    
end

