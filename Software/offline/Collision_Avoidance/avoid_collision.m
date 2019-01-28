function new_delta = avoid_collision(x, delta, relative_locs)

% define tolderance, step size, max iter, etc

% initialize new_delta to max right
% todo run this again but use full left as well, see if they converge in
% the same place

% while we haven't reached max iterations
% determine the gradient

% get the radius of the circle drawn with delta'

% use the circle formula on each landmark to figure out if it's on the path

% if it's within the tolerance, consider it in the way

% use the difference as the input to the derivative of the gaussian

% calculate the derivative and scale according to distance away (prioritize
% closer objects)

% add the difference between delta' and delta for the regularizer

% run gradient step

% once we exhaust iterations, return the new delta

end