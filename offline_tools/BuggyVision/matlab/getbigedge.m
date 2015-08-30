function [] = getbigedge(I_gray)
E= edge(I_gray,'canny',.125);


%removes objects that have fewer then x consecutive pixels
E = bwareaopen(E, 100);


[H, theta, rho] = hough(E);
P = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));

% FillGap:   Positive real scalar value that specifies the distance between
%            two line segments associated with the same Hough transform bin
%            When the distance between the line segments is less the value
%            specified, the houghlines function merges the line segments
%            into a single line segment. Default: 20
%
% MinLength: Positive real scalar value that specifies whether merged lines
%            should be kept or discarded. Lines shorter than the value
%            specified are discarded. Default: 40
lines = houghlines(E,theta,rho,P,'FillGap',20,'MinLength',2);

imshow(E);
hold on

max_len = 0;
for k = 1:length(lines)
    xy = [lines(k).point1; lines(k).point2];
    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    
    % Plot beginnings and ends of lines
    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
    
    % Determine the endpoints of the longest line segment
    len = norm(lines(k).point1 - lines(k).point2);
    if ( len > max_len)
        max_len = len;
        xy_long = xy;
    end
end

% highlight the longest line segment
plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','red');

end