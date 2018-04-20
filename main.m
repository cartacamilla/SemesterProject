clc 
close all
clear 
%%
t = datetime('now');
time_str = datestr(t,'yyyymmddTHHMMSS');

video_processing('Videos\WIN_20180420_16_23_18_Pro.mp4', strcat(time_str,'result.avi'));