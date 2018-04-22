clc
close all
clear all

log = open('20180422T144644test.mat');

%%

FigH = figure('Position', get(0, 'Screensize'));

subplot(2,1,1);hold on;
    plot(log.robot.Error(:,1),'r','LineWidth',1.4)
    plot(log.robot.Error(:,2),'g','LineWidth',1.4)
    plot(log.robot.Error(:,3),'--k','LineWidth',1.4)
    plot(10*log.robot.b1_dot,'--r','LineWidth',1.4)
    xlabel('Time [s]')
    ylabel('Error (inner product)')
    legend('Task 1', 'Task 2', 'Composite','b_1 dot','Location','SouthEast')
    grid on
    hold off;
    
    subplot(2,1,2); hold on;
    stem(log.robot.B_log(1,:),'r')
    stem(log.robot.B_log(2,:),'g')
    ylabel('Beliefs value')
    legend('b_1', 'b_2')
    hold off;
    
saveas(FigH, strcat(log.time_str,'output.png'));