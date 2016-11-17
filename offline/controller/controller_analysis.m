% graph results

clear;
close all;

addpath('./latlonutm/Codes/matlab');
% addpath('./zoharby-plot_google_map');
addpath('./altmany-export_fig');

file = 'localizer_v4.mat';
load(file, 'trajectory', 'P');
save_plot = false;

k = 100;
wmline(trajectory(6,1:k:end), trajectory(7,1:k:end), 'Color', 'r')

heading = trajectory(4,1:k:end);
figure();
hold on;
plot(trajectory(1,1:k:end), trajectory(2,1:k:end))
quiver(trajectory(1,1:k:end), trajectory(2,1:k:end), cos(heading), sin(heading))
hold off;
title(['Heading ', file]);

% plot on google maps
xy = [trajectory(6,1:k:end); trajectory(7,1:k:end)];
fprintf(1, '%5.20f, %5.20f\n', xy);
