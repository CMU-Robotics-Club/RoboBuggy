% graph results

clear;
close all;

addpath('./latlonutm/Codes/matlab');

file = 'localizer_v3.mat';
load(file, 'trajectory', 'P');

k = 100;
t1 = 1476008235676;
t2 = 1476009019782;
t = linspace(0, (t2 - t1)/1000, size(trajectory,2));

trajectory(5,:) = deg2rad( trajectory(5,:) );

Q = cov(trajectory(1:7,:)')

for i = 1:size(trajectory, 1)
	fprintf(1, '%d %5.5f, %5.5f\n', i, mean(trajectory(i,1:k:end)), std(trajectory(i,1:k:end)));
end

fprintf(1, '\n');
for i = 1:size(trajectory, 1)
	fprintf(1, '%d %5.5f, %5.5f\n', i, mean(trajectory(i,1:k)), std(trajectory(i,1:k)));
end
