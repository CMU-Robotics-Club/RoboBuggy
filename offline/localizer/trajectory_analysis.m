% graph results

addpath('./latlonutm/Codes/matlab');
% addpath('./zoharby-plot_google_map');
addpath('./altmany-export_fig');

load('localizer_v2.mat', 'trajectory', 'P');
save_plot = false;

k = 1000;
wmline(trajectory(8,1:k:end), trajectory(9,1:k:end), 'Color', 'r')

t1 = 1476008235676;
t2 = 1476009019782;
t = linspace(0, (t2 - t1)/1000, size(trajectory,2));
figure();
plot(t, trajectory(10,:))

% plot(trajectory(8,:), trajectory(9,:), '.r', 'MarkerSize', 20) 
% plot_google_map('maptype','roadmap', 'APIKey','AIzaSyBYLaDqr2QKRCCRAHhrOf21DVJL-kkkyr8')

if save_plot
    set(gcf,'renderer','zbuffer') 
    export_fig('out.jpg')
end

theta = deg2rad( trajectory(5,1:k:end) );
figure();
hold on;
plot(trajectory(1,1:k:end), trajectory(2,1:k:end))
quiver(trajectory(1,1:k:end), trajectory(2,1:k:end), cos(theta), sin(theta))
hold off;

