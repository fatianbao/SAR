close all;
clear all;
clc;
clf;
%��ɫ����
box on;
%��ͼ����
Nt=500;
base.x=3;
base.y=4;
plane1=Aircraft(3,1,4,6); 
plane1.base=base;
position_xy = randi(130,1,2);
tg=Crashed_ship(position_xy(1)+20,position_xy(2)+20);
axis([0,200,0,200]);
xlabel('X��');
ylabel('Y��');
hold on;
plot(tg.x,tg.y,'o')
hold on
%���ܵı߿�
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
    if (found==0)%δ����Ŀ��
    if flag==false&&tg.find==false%ǰ�����յص�
        plot(plane1.x,plane1.y,'w+')
        plane1=plane1.crusie;
        plot(plane1.x,plane1.y,'bl+')
        pause(0.02)
    elseif flag==true&&tg.find==false%ִ����������
        count_planefind=i*0.02; 
        plot(plane1.x,plane1.y,'w+')
        plane1=plane1.sectorsearch;        
 %       plane1=plane1.goback;
        plot(plane1.x,plane1.y,'bl+')
        pause(0.02)
        tg.find=true;
    else%ִ����������
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
