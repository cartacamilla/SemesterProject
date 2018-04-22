clc 
close all
clear 
%%
t = datetime('now');
time_str = datestr(t,'yyyymmddTHHMMSS');

video_processing('WIN_20180420_15_46_04_Pro.mp4', strcat(time_str,'result.avi'),'hori');