close all;
clear all;
clc;
clf;
xlabel('X��');
ylabel('Y��');
box on;
axis([-2,2,-2,2]);
axis equal;
pause(1);
h=line(NaN,NaN,'marker','o','linesty','-','erasemode','none');
t=6*pi*(0:0.02:1);
for n=1:length(t)
    set(h,'xdata',2*cos(t(1:n)),'ydata',sin(t(1:n)));
    pause(0.05);
    frame=getframe(gcf);
    imind=frame2im(frame);
    [imind,cm] = rgb2ind(imind,256);
    if n==1
         imwrite(imind,cm,'test.gif','gif', 'Loopcount',inf,'DelayTime',1e-4);
    else
         imwrite(imind,cm,'test.gif','gif','WriteMode','append','DelayTime',1e-4);
    end
end
