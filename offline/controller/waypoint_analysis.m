% graph results

clear;
close all;

addpath('../localizer/latlonutm/Codes/matlab');

file = 'controller_v2.mat';
load(file, 'trajectory');
show_maps = false;

load('./waypoints.mat');
[x, y, zone] = ll2utm(logs);
desired = [x y];

size(desired)
size(trajectory)

for k=1:(length(desired) - 1)
    sumdist = norm(desired(k,:) - desired(k+1,:));
end
avgdist = sumdist ./ length(desired)

heading = trajectory(4,1:k:end);
heading = rad2deg(heading);
figure();
hold on;
plot(1:length(heading), heading)
title(['Heading ', file]);

figure();
plot(1:length(trajectory), trajectory(8, 1:end))
title('Control input');

% plot on google maps
if show_maps
    xy = [trajectory(6,1:k:end); trajectory(7,1:k:end)];
    fprintf(1, '%5.20f, %5.20f\n', xy);
end
