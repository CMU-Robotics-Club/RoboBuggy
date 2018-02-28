% graph results

clear;
close all;

addpath('../localizer/latlonutm/Codes/matlab');
show_maps = false;

load('./waypoints_course.mat');
[x, y, zone] = ll2utm(logs);
desired = [x y];

size(desired)

sumdist = 0;
for k=1:(length(desired) - 1)
    sumdist = sumdist + norm(desired(k,:) - desired(k+1,:));
end
avgdist = sumdist ./ length(desired)

% figure();
% plot(1:length(trajectory), trajectory(8, 1:end))
% title('Control input');

% plot on google maps
if show_maps
    xy = [trajectory(6,1:k:end); trajectory(7,1:k:end)];
    fprintf(1, '%5.20f, %5.20f\n', xy);
end
