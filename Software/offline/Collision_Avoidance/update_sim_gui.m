function update_sim_gui(figure_num, obstacles, goal, prev_locs, x)

figure(figure_num);
clf;
hold on
scatter(obstacles(:, 1), obstacles(:, 2), 36, 'b', 'o', 'filled');
scatter(goal(:,1), goal(:,2), 36, 'g', 'x');
scatter(prev_locs(:, 1), prev_locs(:, 2), 36, 'k', '+');
scatter(x(1), x(2), 36, 'r', '+');

end