classdef Crashed_ship<SAR_object
    %UNTITLED5 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        find;
        water_speed;
        found;
    end
    
    methods
        function obj=Crashed_ship(a,b)
            obj.x=a;
            obj.y=b;
            obj.find=false;
        end
        function obj=Drifting(obj,dir)
            %dir= [unifrnd(-1,1),unifrnd(-1,1)]/norm([unifrnd(-1,1),unifrnd(-1,1)]);
            obj.x=obj.x+dir(1);
            obj.y=obj.y+dir(2);
        end
        function flag=Get_Found(obj,plane)
            dx=(plane.x-obj.x)^2;
            dy=(plane.y-obj.y)^2;
            if sqrt(dx+dy)<plane.range
                flag=1;
            else
                flag=0;
            end 
        end
    end 
end

