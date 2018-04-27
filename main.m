clc 
close all
clear 
%%
t = datetime('now');
time_str = datestr(t,'yyyymmddTHHMMSS');

reply = input('Do you want to save the workspace? Y/N:','s');

video_processing('Videos\WIN_20180420_15_46_04_Pro.mp4',...
                 strcat(time_str,'result.avi'),'hori',reply);