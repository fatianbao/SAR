classdef Aircraft<SAR_object
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        range;%探测范围
        speed;
        tg;
        base;
        find;
        th;
        itt;%扇形
        thmean ;
        thspan; % 扇形搜索变量
        
    end
    methods
        function obj=Aircraft(a,b,c,d)%构造函数
            obj.range=a;
            obj.speed=b;
            obj.x=c;
            obj.y=d;
            obj.th=0;
            obj.itt=1;
            obj.thmean = 60/180*pi;
            obj.thspan = 60/180*pi; % 扇形搜索变量
        end
        function h=judge(obj)%目标位置是否在搜索范围内
            dx=(obj.tg.x-obj.x)^2;
            dy=(obj.tg.y-obj.y)^2;
            if sqrt(dx+dy)<obj.range
                h=true;
            else
                h=false;
            end
        end
        function obj=crusie(obj)%巡航
            vxy=[(obj.tg.x-obj.x),(obj.tg.y-obj.y)];
            vxy=obj.speed*vxy/norm(vxy);
            obj.x=obj.x+vxy(1);
            obj.y=obj.y+vxy(2);
        end
        function obj=goback(obj)%返回基地
            vxy=[(obj.base.x-obj.x),(obj.base.y-obj.y)];
            if norm(vxy)>=obj.speed
                vxy=obj.speed*vxy/norm(vxy);
                obj.x=obj.x+vxy(1);
                obj.y=obj.y+vxy(2);
            else
                obj.x=obj.base.x;
                obj.y=obj.base.y;
            end
        end
        function obj=sectorsearch(obj)
            x0=obj.tg.x;
            y0=obj.tg.y;
            r = 20;
            if obj.itt==1
                %第一象限扇形
                if abs(obj.y-y0)<r&&obj.th==0%扇形上升边
                    vxy=r+y0-obj.y;
                    if vxy>=obj.speed
                        obj.y=obj.y+obj.speed;
                    else
                        obj.y=r+y0;
                        obj.th=obj.thmean+1/2*obj.thspan;
                    end
                elseif obj.th>obj.thmean-1/2*obj.thspan%扇形弧段
                    vth=obj.speed/r;
                    obj.th=obj.th-vth;
                    obj.x = r*cos(obj.th)+x0;
                    obj.y = r*sin(obj.th)+y0;
                else
                    vxy=[(x0-obj.x),(y0-obj.y)];%扇形斜边
                    if norm(vxy)>=obj.speed
                        vxy=obj.speed*vxy/norm(vxy);
                        obj.x=obj.x+vxy(1);
                        obj.y=obj.y+vxy(2);
                    else
                        obj.x=x0;
                        obj.y=y0;
                        obj.itt=obj.itt+1;
                        obj.th=-1;
                    end
                end
            elseif obj.itt==2
                %第二象限扇形
                obj.thmean=-30/180*pi;
                obj.thspan=60/180*pi;
                if abs(obj.x-x0)<=r&&obj.th==-1%扇形右移边
                    vxy=r+x0-obj.x;
                    if vxy>=obj.speed
                        obj.x=obj.x+obj.speed;
                    else
                        obj.x=r+x0;
                        obj.th=obj.thmean+1/2*obj.thspan;
                    end
                elseif obj.th>obj.thmean-1/2*obj.thspan%扇形弧段
                    vth=obj.speed/r;
                    obj.th=obj.th-vth;
                    obj.x = r*cos(obj.th)+x0;
                    obj.y = r*sin(obj.th)+y0;
                else
                    vxy=[(x0-obj.x),(y0-obj.y)];%扇形斜边
                    if norm(vxy)>=obj.speed
                        vxy=obj.speed*vxy/norm(vxy);
                        obj.x=obj.x+vxy(1);
                        obj.y=obj.y+vxy(2);
                    else
                        obj.x=x0;
                        obj.y=y0;
                        obj.itt=obj.itt+1;
                        obj.th=-1;
                    end
                end
                elseif obj.itt==3
                %第三象限扇形
                obj.thmean=-120/180*pi;
                obj.thspan=60/180*pi;
                if abs(obj.y-y0)<=r&&obj.th==-1%扇形下降边
                    vxy=-r+y0-obj.y;
                    if abs(vxy)>=obj.speed
                        obj.y=obj.y-obj.speed;
                    else
                        obj.y=-r+y0;
                        obj.th=obj.thmean+1/2*obj.thspan;
                    end
                elseif obj.th>obj.thmean-1/2*obj.thspan%扇形弧段
                    vth=obj.speed/r;
                    obj.th=obj.th-vth;
                    obj.x = r*cos(obj.th)+x0;
                    obj.y = r*sin(obj.th)+y0;
                else
                    vxy=[(x0-obj.x),(y0-obj.y)];%扇形斜边
                    if norm(vxy)>=obj.speed
                        vxy=obj.speed*vxy/norm(vxy);
                        obj.x=obj.x+vxy(1);
                        obj.y=obj.y+vxy(2);
                    else
                        obj.x=x0;
                        obj.y=y0;
                        obj.itt=obj.itt+1;
                        obj.th=-1;
                    end
                end
                elseif obj.itt==4
                %第四象限扇形
                obj.thmean=150/180*pi;
                obj.thspan=60/180*pi;
                if abs(obj.x-x0)<=r&&obj.th==-1%扇形上升边
                    vxy=-r+x0-obj.x;
                    if abs(vxy)>=obj.speed
                        obj.x=obj.x-obj.speed;
                    else
                        obj.x=-r+x0;
                        obj.th=obj.thmean+1/2*obj.thspan;
                    end
                elseif obj.th>obj.thmean-1/2*obj.thspan%扇形弧段
                    vth=obj.speed/r;
                    obj.th=obj.th-vth;
                    obj.x = r*cos(obj.th)+x0;
                    obj.y = r*sin(obj.th)+y0;
                else
                    vxy=[(x0-obj.x),(y0-obj.y)];%扇形斜边
                    if norm(vxy)>=obj.speed
                        vxy=obj.speed*vxy/norm(vxy);
                        obj.x=obj.x+vxy(1);
                        obj.y=obj.y+vxy(2);
                    else
                        obj.x=x0;
                        obj.y=y0;
                        obj.itt=obj.itt+1;
                        obj.th=-1;
                    end
                end
            end
            
        end
        
    end
end



