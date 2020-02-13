close all;
clear all;
clc;
clf;
%白色背景
box on;
%绘图区域
Nt=500;
base.x=3;
base.y=4;
plane1=Aircraft(3,1,4,6); 
plane1.base=base;
position_xy = randi(130,1,2);
tg=Crashed_ship(position_xy(1)+20,position_xy(2)+20);
axis([0,200,0,200]);
xlabel('X轴');
ylabel('Y轴');
hold on;
plot(tg.x,tg.y,'o')
hold on
%四周的边框
plane1.tg=tg; 
lx=unifrnd(-1,1);    
ly=unifrnd(-1,1);
dir= 0.05*[lx,ly]/norm([lx,ly]);
found=0;
for i=1:Nt
    flag=plane1.judge;
    plot(tg.x,tg.y,'wo')
    tg=Drifting(tg,dir);
    plot(tg.x,tg.y,'blo')
    hold on
    found=found+tg.Get_Found(plane1);
    if (found==0)%未发现目标
    if flag==false&&tg.find==false%前往遇险地点
        plot(plane1.x,plane1.y,'w+')
        plane1=plane1.crusie;
        plot(plane1.x,plane1.y,'bl+')
        pause(0.02)
    elseif flag==true&&tg.find==false%执行扇形搜索
        count_planefind=i*0.02; 
        plot(plane1.x,plane1.y,'w+')
        plane1=plane1.sectorsearch;        
 %       plane1=plane1.goback;
        plot(plane1.x,plane1.y,'bl+')
        pause(0.02)
        tg.find=true;
    else%执行扇形搜索
        plot(plane1.x,plane1.y,'w+')
        plane1=plane1.sectorsearch;
%         plane1=plane1.goback;
        plot(plane1.x,plane1.y,'bl+')
        pause(0.02)
    end
    else 
        plot(plane1.x,plane1.y,'w+')
%        plane1=plane1.sectorsearch;
         plane1=plane1.goback;
        plot(plane1.x,plane1.y,'bl+')
        pause(0.02)
    end
end
