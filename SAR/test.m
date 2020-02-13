clc;
clf;
clear all;
global obstacle;
obstacle=[];
global fls;
global auvpos;
fls=cell(1,1);
auvpos=cell(1,1);
insight_point=cell(1,1);
MaximumRange = 150; % 最大探测范围
Vl=zeros(2000,2000);%环境代价矩阵
Time_AllSteps = 50; %额定步数
Y = zeros(Time_AllSteps,6); %状态变量
global subregion;
subregion=[];
sub_obs_coor=[300,900;500,1300;500,1500;700,1300;700,1100;900,700;900,500;
   1100,900;1100,700;1100,500;1300,1500;1300,1300;1300,700;
    1300,900;1300,500;1500,1300;1500,700;1500,500;1500,1500];%障碍物占据的子区域中心点坐标
n1=1;
n2=1;
sub_area=[];
pp=0;
sub_pp_number=0;
for i=1:5
 for j=1:5
  p_obs=0;
  for n=1:2
   for p=1:2
    pp=0;
    for m=1:size(sub_obs_coor,1)
     if((200+(i-1)*400+(-1)^n*100~=sub_obs_coor(m,1))||(200+(j-1)*400+(-1)^p*100~=sub_obs_coor(m,2)))
      p_obs=1;
     else
      pp=1;
     break;
     end
    end
    if(pp==0)
     sub_pp_number=sub_pp_number+1;
     sub_area=[sub_area,200+(i-1)*400+(-1)^n*100,200+(j-1)*400+(-1)^p*100,0];
    end
   end
  end
  if(p_obs~=0)
   mm=size(sub_area,2);
   if(mm>0)
    subregion(n1,1:5+mm) = [n1,200+(i-1)*400,200+(j-1)*400,0,0,sub_area];
    subregion(n1,18)=sub_pp_number;
    sub_pp_number=0;
    sub_area=[];
    n1=1+n1;
   end
  end
 end
end
global AUV_status;
AUV_status=cell(1,1);
AUV_Angle(:,1)=[0,0,0,0,0,0,0,0,0,0];%AUV的初始艏向角
auv_num(:,1)=[2,2,2,2,2,2,2,2,2,2];
global num_target;
global number_target;
global pos_target;
global pos_tar1;
global sub_auv;
global sublock_area;
global auv_hunt_flag;
global tracking_tar;
global auv_collaborators;
auv_collaborators=zeros(2,3);
[num_target,pos_target,Obstacles]=Make_obstacles4();%障碍物边界
pos_tar1=[pos_target',zeros(size(pos_target,2),1)];
number_target=num_target;
button = 0;
mouse_click=0;
a_n = 1;
while button ~= 3
[x,y,button]=ginput(1); %十字光标
pin= ~isempty(x);
if (pin==1 && button ~= 3)
 mouse_click=mouse_click+1;
  plot(x,y,'mo')
  AUV_status{1}{a_n}(:,1)=[20,0,0,x,y,AUV_Angle(a_n,1)];
  a_n = a_n+1;
end
end
hold off;
AUV_num=mouse_click;
AUV_AngTemp=AUV_Angle(1:mouse_click,1);
%******视线导引的切换点的半径*********
R=8;
n=ones(AUV_num,1);
angle_v=zeros(AUV_num,1);
flag_obsfind=zeros(AUV_num,1);
line_ok=zeros(AUV_num,1);
sub_flag = zeros(AUV_num,1);
sub_ok = zeros(AUV_num,1);
task_time = zeros(AUV_num,1);
sub_counter = 0;
flag_a=ones(AUV_num,1);
excute_task=zeros(AUV_num,1);
to_sub=zeros(AUV_num,1);
task_do=zeros(AUV_num,1);
tracking_tar=zeros(AUV_num,2);
task_continue=ones(AUV_num,1);
lock_area=zeros(AUV_num,4);
auv_task_start=zeros(AUV_num,2);
auv_hunt_flag=zeros(AUV_num,1);
allsub_task_over=0;
end_flag=0;
stop_sub=zeros(AUV_num,2);
taskovercomingflag=0;
change_flag=0;
dy_search_over=0;
flag_changesub=zeros(AUV_num,3);
sub_auv=zeros(AUV_num,1);
sublock_area=zeros(AUV_num,3);
obs_auv_PosAngleDis=cell(1,1);
sub_n=8;
sub2overflag=1;
line_color=[0,0,1;0,1,0;1,0,0;1,1,0;0,1,1;1,0,1;1,1,1];
global pid_err;
global pid_errobs;
global dynamic_mess;
global Dynamic_pos;
global hunt_time;
global dy_n;
global auv_group;
global vanish_flag;
vanish_flag=zeros(2,3);
auv_group=zeros(6,4);
dy_n=ones(AUV_num,1);
Dynamic_pos=zeros(2,3);
dy_tar_auv_num=zeros(2,1);
hunt_time=zeros(2,1);
pid_err=cell(1,3);
pid_errobs=cell(1,1);
dynamic_mess=cell(1,1);
dy_tar_num=0;
for i=1:AUV_num
 pid_err{1}{i}(1:3,1)=0;
 pid_errobs{1}{i}(1:3,1)=0;
 obs_auv_PosAngleDis{1}{i}=[];
 AUV_PosTemp(:,i)=AUV_status{1}{i}(4:5,1);%AUV位置临时变量
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%******视线导引控制PID参数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global K_p1;
global K_i1;
global K_d1;
% K_p1 =1.0; K_i1 = 0.525; K_d1 =0; %艏向控制
K_p1 =0.35; K_i1 = 0.85; K_d1 =0; %艏向控制
global K_p1obs;
global K_i1obs;
global K_d1obs;
% K_p1obs =0.5; K_i1obs = 3.5; K_d1obs =0;
K_p1obs =0.1; K_i1obs = 0.225; K_d1obs =0;%1 0.625
%%%%%%%%%%%%%%%%%%%
%随机动目标位置生成
x_rt=800;%随机目标位置初始化
y_rt=1600;
global x_sin;
global x_change_flag;
global tar0_Angle;
x_sin=400;
x_change_flag=0;
tar0_Angle=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Env_handle=environment_buliding4(pos_target);%环境构建
DrawAUV(AUV_PosTemp,AUV_Angle,AUV_num);
for step=1:1:4000%最大执行步数
% step=1;
% while(1)
 t_obs = 0.1;
 T1(step,1)=0.1*step;
 [OBS_point,flag_obs]=FLS_obsmeasure(Obstacles,AUV_PosTemp,AUV_AngTemp);
 [Dynamic_pos(:,1:2),dynamic_pos,dynamic_angle]=RandTarget(x_rt,y_rt);
 for i=1:length(AUV_AngTemp)
  [search_tar_over]=target_search(AUV_PosTemp(:,i),AUV_AngTemp(i,1));
  if(dy_tar_num<2)
   [finddynamic_ok,angle_v(i,1),vanish,dy_tar_n,dynamic_tar_pos,find_tar_auv,hunt_group_done]=DynamicTargetSearchFun(AUV_PosTemp,AUV_AngTemp(i,1),Dynamic_pos,i);
  end
  if(search_tar_over==1)%静态目标和动态目标搜索完毕
   if(dy_tar_num==2)
    break;
   else
    dy_search_over=1;
    subregion(:,4:5)=0;
   end
  end
 
  if(flag_obs(i,1)==1&&line_ok(i,1)==0) %上一个时间AUV完成了导引避障
   flag_a(i,1)=0;
   finddynamic_ok=0;
%    pid_err{1}{i}(:,1)=0;
   flag_obsfind(i,1)=1;%第i个AUV遇到障碍物
   AUV_status{1}{i}(1,auv_num(i,1)-1)=20;%有障碍减速
   insight_point{1}{i}(:,1)=AUV_status{1}{i}(4:5,auv_num(i,1)-1);%视线导引起点为当前AUV的位置
   insight_point{1}{i}(:,2)= OBS_point(i,:)';   %终点为推算的避障点
   obs_auv_PosAngleDis{1}{i}(n(i,1),:)=OBS_point(i,:);
   n(i,1)= n(i,1)+1;
  else if((flag_obs(i,1)==1&&line_ok(i,1)==1)) %当前时刻检测到障碍物但是上一时刻未完成导引避障
    flag_a(i,1)=0;
    finddynamic_ok=0;
    flag_obsfind(i,1)=1;%第i个AUV遇到障碍物
%     AUV_status{1}{i}(1,auv_num(i,1)-1)=20;%有障碍减速
   else if(flag_obs(i,1)==0&&line_ok(i,1)==1)%当前时刻未检测到障碍物但是上一时刻未完成导引避障
    flag_a(i,1) =0;
    finddynamic_ok=0;
    flag_obsfind(i,1)=1;
    else%当前时刻AUV未检测到障碍物，上一时刻的导引已经完成
     flag_a(i,1) =1;
%      pid_err{1}{i}(:,1)=0;
     flag_obsfind(i,1)=0;%第i个AUV未遇到障碍物
 %     angle_v(i,1)=0;  %不需要避障的，角速度为0
     AUV_status{1}{i}(1,auv_num(i,1)-1)=40;%无障碍加速
    end
   end
  end
  if(flag_obsfind(i,1)==1)%判断第i个AUV需要视线导引避障
   line_ok(i,1)=1;
   [r,error,dt,emix]=insight_line(AUV_status{1}{i}(:,auv_num(i,1)-1),insight_point{1}{i}(:,1),insight_point{1}{i}(:,2),i);%直线视线导引控制避障
   if(dt<=R)%AUV满足线段切换条件
    flag_a(i,1)=1;
    flag_obsfind(i,1)=0;%重启声呐检测障碍物
    angle_v(i,1)=0;
    pid_err{1}{i}(:,1)=0;
    line_ok(i,1)=0;
   else
    angle_v(i,1)=r;
   end
  end
%   %%%%%%%%%%AUV走向分配的子区域过程中或执行子区域任务与避障的临时切换机制%%%%%%%%
%   %如果在AUV走向分配的子区域过程中或执行子区域任务时需要避障，
%   %那么进行避障，结束后继续执行被中断的过程
%   %而不是再重新分配子区域，因为被中断的过程还没有完成
%   %%%%%%%%%%%%%%%%%%%切换机制代码区%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if(flag_a(i,1)==0)
   if(task_do(i,1)==1)
    excute_task(i,1)=1;%在执行子区域任务时被避障事件中断
   else if(sub_ok(i,1)==0)
     to_sub(i,1)=1;%在走向分配的子区域是被避障事件中断
    end
   end
%    if(tracking_tar(i,1)==1)%目标围捕过程中发生避障事件
%     excute_task(i,1)=0;
%     to_sub(i,1)=0;%前期执行的走向子区域和子区域搜索都作废，先避障，
%         %如果动态目标不在视域内，完事再重新进行子区域分配;如果在视域内继续进行目标围捕
%    end
  end
  if(finddynamic_ok==1)%发现动态目标，进行跟踪
   tracking_tar(i,1)=1;%当前AUV处于动目标围捕过程
   tracking_tar(i,2)=dy_tar_n;%当前AUV围捕的目标编号
   flag_a(i,1)=0;%进行动目标围捕不执行子区域分配
   if(find_tar_auv~=0)
    dy_tar_auv_num(dy_tar_n,1)=find_tar_auv;
    organizing_auv( AUV_PosTemp,find_tar_auv,dy_tar_n);%一个AUV发现目标后确定队形的其他AUV编号，组成围捕队形
    tracking_tar(auv_collaborators(dy_tar_n,2),1)=1;%当前AUV处于动目标围捕过程
    tracking_tar(auv_collaborators(dy_tar_n,2),2)=dy_tar_n;%当前AUV围捕的目标编号
    flag_a(auv_collaborators(dy_tar_n,2),1)=0;
    tracking_tar(auv_collaborators(dy_tar_n,3),1)=1;%当前AUV处于动目标围捕过程
    tracking_tar(auv_collaborators(dy_tar_n,3),2)=dy_tar_n;%当前AUV围捕的目标编号
    flag_a(auv_collaborators(dy_tar_n,3),1)=0;
    sub_auv(find_tar_auv)=0;%进行目标围捕过程中，以前的子区域锁定没到达的取消，取消AUV锁定子区域的标志
    sub_auv(auv_collaborators(dy_tar_n,2))=0;
    sub_auv(auv_collaborators(dy_tar_n,3))=0;
    if(lock_area(find_tar_auv,1)~=0)
     if(subregion(lock_area(find_tar_auv,1),lock_area(find_tar_auv,2))==0.5)
      subregion(lock_area(find_tar_auv,1),lock_area(find_tar_auv,2))=0;
     end
    end
    if(lock_area(auv_collaborators(dy_tar_n,2),1))
     if(subregion(lock_area(auv_collaborators(dy_tar_n,2),1),lock_area(auv_collaborators(dy_tar_n,2),2))==0.5)
      subregion(lock_area(auv_collaborators(dy_tar_n,2),1),lock_area(auv_collaborators(dy_tar_n,2),2))=0;
     end
    end
    if(lock_area(auv_collaborators(dy_tar_n,3),1))
     if(subregion(lock_area(auv_collaborators(dy_tar_n,3),1),lock_area(auv_collaborators(dy_tar_n,3),2))==0.5)
      subregion(lock_area(auv_collaborators(dy_tar_n,3),1),lock_area(auv_collaborators(dy_tar_n,3),2))=0;
     end
    end
   end
  end
  if(vanish==1)%动态目标已经围捕成功
   vanish=0;
   flag_a(dy_tar_auv_num(dy_tar_n,1),1)=1;%三个AUV解散队形
   tracking_tar(dy_tar_auv_num(dy_tar_n,1),1)=0;
   tracking_tar(dy_tar_auv_num(dy_tar_n,1),2)=0;
   flag_a(auv_collaborators(dy_tar_n,2),1)=1;
   tracking_tar(auv_collaborators(dy_tar_n,2),1)=0;
   tracking_tar(auv_collaborators(dy_tar_n,2),2)=0;
   flag_a(auv_collaborators(dy_tar_n,3),1)=1;
   tracking_tar(auv_collaborators(dy_tar_n,3),1)=0;
   tracking_tar(auv_collaborators(dy_tar_n,3),2)=0;
   auv_hunt_flag(dy_tar_auv_num(dy_tar_n,1))=0;
   auv_hunt_flag(auv_collaborators(dy_tar_n,2))=0;
   auv_hunt_flag(auv_collaborators(dy_tar_n,3))=0;
   sub_flag(dy_tar_auv_num(dy_tar_n,1))=0;
   excute_task(dy_tar_auv_num(dy_tar_n,1),1)=0;
   to_sub(dy_tar_auv_num(dy_tar_n,1),1)=0;
   sub_flag(auv_collaborators(dy_tar_n,2),1)=0;
   excute_task(auv_collaborators(dy_tar_n,2),1)=0;
   to_sub(auv_collaborators(dy_tar_n,2),1)=0;
   sub_flag(auv_collaborators(dy_tar_n,3),1)=0;
   excute_task(auv_collaborators(dy_tar_n,3),1)=0;
   to_sub(auv_collaborators(dy_tar_n,3),1)=0;
   dy_tar_num=dy_tar_num+1;
  end
  %%%%%%%%任务子区域分配代码区域%%%%%%%%%%%%%%%%%%%
  if(flag_a(i,1)==1)
   if(sub_flag(i,1)==0&&to_sub(i,1)==0&&excute_task(i,1)==0)%无锁定的子区域,无中断的走向子区域和执行任务过程发生
    if(end_flag==0)
     [lock_area(i,:), end_comingflag,sub2flag] = Search_layer(AUV_PosTemp,i,dy_search_over); %分配锁定的子区域进行目标搜索
     if(end_comingflag==1)
      end_flag=1;
      taskovercomingflag=1;
     end
     sub_flag(i,1)=1;
     AUV_status{1}{i}(1,auv_num(i,1)-1)=40;
     auv_task_start(i,:)=AUV_status{1}{i}(4:5,auv_num(i,1)-1);
     stop_sub(i,:)=lock_area(i,3:4);
     %检测当前AUV在所锁定的子区域范围中
     if(sqrt((AUV_PosTemp(1,i)-lock_area(i,3))^2+(AUV_PosTemp(2,i)-lock_area(i,4))^2)<=100)
%       subregion(lock_area(i,1),5)=0.5;%被分配的AUV进入相应的区域，但还没有完成任务
      sub_ok(i,1)=1;%当前AUV在所分配的子区域
      task_do(i,1)=1;
     else
      sub_ok(i,1)=0;
      task_do(i,1)=0;
     end
    end 
   end
   if(taskovercomingflag==1)%%结束全部子区域搜索来临了
     [freeAUV_help_area,sub2overflag]=cooperation_auvs(AUV_PosTemp,i,dy_search_over);%区域一次性去全部分配完事，接下来就是到需要帮助的区域执行任务
     if(sub2overflag==0)
     break;
     end
     if(flag_changesub(i,2)>=1)
      if(freeAUV_help_area(1,1)==flag_changesub(i,1)&&freeAUV_help_area(1,2)==flag_changesub(i,2))
       change_flag=2;     
      else
       change_flag=1;
       flag_changesub(i,1)=freeAUV_help_area(1,1);
       flag_changesub(i,2)=freeAUV_help_area(1,2);
       sub_ok(i,1)=0;%当前AUV在所分配的子区域freeAUV_help_area(1,2)~=flag_changesub(i,1)
       task_do(i,1)=0;
       task_time(i,1)=0;
       excute_task(i,1)=0; 
      end
     else
      change_flag=0;
      flag_changesub(i,1)=freeAUV_help_area(1,1);
      flag_changesub(i,2)=freeAUV_help_area(1,2);
      flag_changesub(i,3)=1;
     end
     if((change_flag==0)||(change_flag==1))
      lock_area(i,:)=freeAUV_help_area(1,:);
      auv_task_start(i,:)=AUV_status{1}{i}(4:5,auv_num(i,1)-1);
      stop_sub(i,:)=freeAUV_help_area(1,3:4);
      AUV_status{1}{i}(1,auv_num(i,1)-1)=40;
      pid_errobs{1}{i}(:,1)=0;
      if(sqrt((AUV_status{1}{i}(4,auv_num(i,1)-1)-stop_sub(i,1))^2+(AUV_status{1}{i}(5,auv_num(i,1)-1)-stop_sub(i,2))^2)<=100)
%        subregion(lock_area(i,1),5)=0.5;%被分配的AUV进入相应的区域，但还没有完成任务
       sub_ok(i,1)=1;%当前AUV在所分配的子区域
       task_do(i,1)=1;
      else
       sub_ok(i,1)=0;
       task_do(i,1)=0;
      end
     end
   end
   if(sub_ok(i,1)==0)%没有在分配的子区域中则去到达锁定的范围
    if(to_sub(i,1)==1||task_continue(i,1) ==0)%在走向分配的子区域被中断，改变导引的起始点为避障完成后AUV的坐标
     auv_task_start(i,:)=AUV_status{1}{i}(4:5,auv_num(i,1)-1);
     pid_errobs{1}{i}(:,1)=0;
     to_sub(i,1)=0;
    end
    [r_search,error1,dt1,emix1] = insight_line_obs(AUV_status{1}{i}(:,auv_num(i,1)-1),auv_task_start(i,:)',stop_sub(i,:)',i);
    angle_v(i,1) = r_search;
    if(dt1<=100)%到达了被分配的子区域，则进行目标搜索
     sub_ok(i,1)=1;%当前AUV在所分配的子区域
     task_do(i,1)=1;
     task_continue(i,1) =1;
     to_sub(i,1)=0;
     pid_errobs{1}{i}(:,1)=0;
     angle_v(i,1)=0;
%      subregion(lock_area(i,1),5)=1;%被分配的AUV进入相应的区域，但还没有完成任务
    end
   end
   if(sub_ok(i,1)==1&&task_do(i,1)==1)
    if(excute_task(i,1)==1)%在执行子区域任务时被避障中断，恢复子区域任务
     %AUV完成避障依旧在被分配的子区域中，那么继续执行区域任务
     excute_task(i,1)=0;
     if(sqrt((AUV_status{1}{i}(4,auv_num(i,1)-1)-lock_area(i,3))^2+(AUV_status{1}{i}(5,auv_num(i,1)-1)-lock_area(i,4))^2)<=100)
      task_continue(i,1) =1;
     else%否则完成避障不在分配的子区域中，继续回到被分配的子区域，继续执行剩下的任务
      task_continue(i,1) =0;%不能继续执行子区域任务，要回到子区域中才可以继续执行
     end
    end
    if(task_continue(i,1) ==1)%不是在执行任务时被中断的，继续执行当前子区域任务
   %在分配的子区域范围中，进行目标搜索
   %添加当前AUV在子区域中的执行代码
   %现在检验为原地转圈
%      angle_v(i,1)=0;
%      AUV_status{1}{i}(1,auv_num(i,1)-1)=0;
     task_time(i,1) = 1+task_time(i,1);
     if(task_time(i,1)>=1)
     task_time(i,1)=0;
     sub_flag(i,1)=0;
     task_do(i,1)=0;
     excute_task(i,1)=0;
     angle_v(i,1)=0;
     subregion(lock_area(i,1),18)=subregion(lock_area(i,1),18)-1;
     if(subregion(lock_area(i,1),18)==0)%子区域的四个子区域都搜索完毕了
       subregion(lock_area(i,1),5)=1;%相应的编号区域已经完成了任务
     end
     subregion(lock_area(i,1),lock_area(i,2))=1;
     allsub_task_over = allsub_task_over+1;
%      if(allsub_task_over==size(subregion,1))
%       break;
%      end
     end
    end
   end
  end
  %%%%%%%%%%%%%%%%%%子区域代码结束区域%%%%%%%%%%%%%%%%%%%%%%
  k1=AUV_Model(AUV_status{1}{i}(:,auv_num(i,1)-1),angle_v(i,1));%AUV模型
  AUV_status{1}{i}(:,auv_num(i,1))=AUV_status{1}{i}(:,auv_num(i,1)-1)+k1;
  if(AUV_status{1}{i}(6,auv_num(i,1))>pi)
   AUV_status{1}{i}(6,auv_num(i,1))=AUV_status{1}{i}(6,auv_num(i,1))-2*pi;
  else if(AUV_status{1}{i}(6,auv_num(i,1))<-pi)
   AUV_status{1}{i}(6,auv_num(i,1))=AUV_status{1}{i}(6,auv_num(i,1))+2*pi;
   end
  end
  AUV_PosTemp(1:2,i)=AUV_status{1}{i}(4:5,auv_num(i,1));%当前AUV位置
  AUV_AngTemp(i,1)=AUV_status{1}{i}(6,auv_num(i,1));%当前AUV角度
  auv_num(i,1)=auv_num(i,1)+1;
 end 
 %*******画动态图*******************%
 environment_plot4(pos_target,step,T1(step,1),task_time(:,1)',lock_area,number_target);%环境构建
 DrawAUV(AUV_PosTemp,AUV_AngTemp,AUV_num);%画AUV
 made_dynamictar(dynamic_pos,dynamic_angle,2);
 for i=1:AUV_num
  if(isempty(obs_auv_PosAngleDis{1}{i})==0)
   plot(obs_auv_PosAngleDis{1}{i}(:,1),obs_auv_PosAngleDis{1}{i}(:,2),'mo');
  end
  line(AUV_status{1}{i}(4,:),AUV_status{1}{i}(5,:),'linewidth',1.5,'color',line_color(i,:));%绘制AUV实时轨迹
 end
 hold off;
 drawnow;
 step = step + 1;
 if(search_tar_over==1&&dy_tar_num==2||sub2overflag==0)%子区域全部分配完就结束所有任务，暂时先这样。后期还会更
 break;
 end
end

% figure(4);clf;
% plot(T,error,'r','LineWidth',2);grid on;xlabel('仿真时间');ylabel('误差(m)');
% figure(5);clf;
% heading_angle(:,1)=AUV_status{1}{1}(6,:)*180/pi;
% heading_speed(:,1)=AUV_status{1}{1}(3,:)*180/pi;
% subplot(2,1,1);
% plot(T1,heading_speed,'b','LineWidth',2);grid on;xlabel('仿真时间');ylabel('角速度(°)');
% subplot(2,1,2);
% plot(T1,heading_angle,'b','LineWidth',2);grid on;xlabel('仿真时间');ylabel('艏向(°)');
