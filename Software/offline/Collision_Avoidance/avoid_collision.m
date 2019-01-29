function new_delta = avoid_collision(x, delta, relative_locs)

% define tolerance, step size, max iter, etc
max_iter = 200;
step_size = 0.01;
distance_tolerance = 0.5;
delta_circle_tol = 0.1;
wheelbase = 1;
derivative_step_dist = 0.001;

% initialize new_delta to max right
% todo run this again but use full left as well, see if they converge in
% the same place
new_delta = deg2rad(15);

% while we haven't reached max iterations
% determine the gradient
for i = 1:max_iter

% get the radius of the circle drawn with delta'
% from http://www.davdata.nl/math/turning_radius.html
% unless delta is below the tolerance, then the circle will be too big to
% compute
if abs(delta) > delta_circle_tol
    r = wheelbase * tan(delta);
    
    % use the circle formula on each landmark to figure out if it's on the path
    candidate_rs = sqrt(sum(relative_locs .^2));
    offsets = candidate_rs - ones(size(candidate_rs))*r;
    offsets = (abs(offsets) < distance_tol);

    % if it's within the tolerance, consider it in the way
else
    % figure out if the landmarks are right in front
    offsets = relative_locs(relative_locs(:,2) < distance_tolerance, 2);
    
end

% use the difference as the input to the derivative of the gaussian
pdf_val = normpdf(offsets);
pdf_val_plus = normpdf(offsets+derivative_step_dist);
derivatives = (pdf_val - pdf_val_plus) ./ derivative_step_dist;

gradient_step = sum(derivatives);

% TODO calculate the derivative and scale according to distance away (prioritize
% closer objects)

% add the difference between delta' and delta for the regularizer
gradient_step = gradient_step + abs(new_delta - delta);

% run gradient step
new_delta = new_delta + step_size * gradient_step;

end

% once we exhaust iterations, return the new delta

end