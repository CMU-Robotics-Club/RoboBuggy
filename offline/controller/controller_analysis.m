% graph results

clear;
close all;

addpath('../localizer/latlonutm/Codes/matlab');
addpath('../localizer/altmany-export_fig');

file = 'controller_v1.mat';
load(file, 'trajectory');
save_plot = false;
show_maps = true;

k = 1000;
if show_maps
    wmline(trajectory(6,1:k:end), trajectory(7,1:k:end), 'Color', 'r')
end

heading = trajectory(4,1:k:end);
figure();
hold on;
plot(trajectory(1,1:k:end), trajectory(2,1:k:end))
quiver(trajectory(1,1:k:end), trajectory(2,1:k:end), cos(heading), sin(heading))
hold off;
title(['Heading ', file]);

% plot on google maps
if show_maps
    xy = [trajectory(6,1:k:end); trajectory(7,1:k:end)];
    fprintf(1, '%5.20f, %5.20f\n', xy);
end
