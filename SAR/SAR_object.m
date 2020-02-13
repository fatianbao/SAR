classdef SAR_object
    %UNTITLED6 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        x;
        y;
    end
    
    methods
        % 获取函数get.m
        function val = get(d,prop_name)  %function 返回变量=函数名（输入变量）
            switch prop_name  %变量名可能是x，也可能是y
                case 'x'
                    val=d.x;
                case 'y'
                    val=d.y;
                otherwise
                    error([prop_name,'is not a valid list property']);
            end
        end
        %set.m
        function d = set(d,varargin)  %function 返回变量=函数名（输入变量）
            argin=varargin;
            while length(argin)>=2
                prop=argin{1};
                val=argin{2};
                argin=argin(3:end);
                switch prop
                    case 'x'
                        d.x=val;
                    case 'y'
                        d.y=val;
                    otherwise
                        error('Asset properties:x,y');
                end
            end
        end
        
    end
end

