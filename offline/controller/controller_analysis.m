% graph results

clear;
close all;

addpath('../localizer/latlonutm/Codes/matlab');
addpath('../localizer/altmany-export_fig');

file = 'controller_v2.mat';
load(file, 'trajectory');
save_plot = false;
show_maps = false;

load('./waypoints.mat');
[x, y, zone] = ll2utm(logs);
desired = [x y];


k = 1000;
if show_maps
    wmline(trajectory(6,1:k:end), trajectory(7,1:k:end), 'Color', 'r')
end

heading = trajectory(4,1:k:end);
figure();
hold on;
plot(trajectory(1,1:k:end), trajectory(2,1:k:end))
quiver(trajectory(1,1:k:end), trajectory(2,1:k:end), cos(heading), sin(heading))
plot(desired(:,1), desired(:,2), 'g')
hold off;
title(['Heading ', file]);

figure();
p = k/20;
plot(1:p:size(trajectory,2), trajectory(8,1:p:end))
title('Control input');

% plot on google maps
if show_maps
    xy = [trajectory(6,1:k:end); trajectory(7,1:k:end)];
    fprintf(1, '%5.20f, %5.20f\n', xy);
end
