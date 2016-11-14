% graph results

clear;
close all;

addpath('./latlonutm/Codes/matlab');
% addpath('./zoharby-plot_google_map');
addpath('./altmany-export_fig');

file = 'localizer_v3.mat';
load(file, 'trajectory', 'P');
save_plot = false;

k = 1000;
wmline(trajectory(8,1:k:end), trajectory(9,1:k:end), 'Color', 'r')

theta = deg2rad( trajectory(5,1:k:end) );
figure();
hold on;
plot(trajectory(1,1:k:end), trajectory(2,1:k:end))
quiver(trajectory(1,1:k:end), trajectory(2,1:k:end), cos(theta), sin(theta))
hold off;
title(['Heading ', file]);

% plot on google maps
ss = [trajectory(8,1:k:end); trajectory(9,1:k:end)];
fprintf(1, '%5.20f, %5.20f\n', ss);
