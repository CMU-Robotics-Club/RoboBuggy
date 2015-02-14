% initialize the graph plot

scrsz = get(0,'ScreenSize');
animation = figure(1);

global target h_axes;

SCREEN_WIDTH = scrsz(3)
SCREEN_HEIGHT = scrsz(4)


set(animation,'name','Matlab_hax','Position',[1 1 SCREEN_WIDTH SCREEN_HEIGHT]);
h_axes = axes('Parent',animation,'Units','Pixels','Position',[0 0 SCREEN_WIDTH SCREEN_HEIGHT]);

target = line('Parent',h_axes,'Color', [ 0 1 0 ],'Visible','off','LineWidth',10);

set(h_axes,'visible','on');