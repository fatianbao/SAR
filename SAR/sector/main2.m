clear all
x0 = 0;
y0 = 0;
r = 10;
thmean = 60/180*pi;
thspan = 60/180*pi;
th = linspace(thmean-1/2*thspan,thmean+1/2*thspan,100);
x = r*cos(th)+x0;
y = r*sin(th)+y0;
plot([x,0,x(1)],[y,0,y(1)],'-k');
