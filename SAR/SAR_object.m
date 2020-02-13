classdef SAR_object
    %UNTITLED6 �˴���ʾ�йش����ժҪ
    %   �˴���ʾ��ϸ˵��
    
    properties
        x;
        y;
    end
    
    methods
        % ��ȡ����get.m
        function val = get(d,prop_name)  %function ���ر���=�����������������
            switch prop_name  %������������x��Ҳ������y
                case 'x'
                    val=d.x;
                case 'y'
                    val=d.y;
                otherwise
                    error([prop_name,'is not a valid list property']);
            end
        end
        %set.m
        function d = set(d,varargin)  %function ���ر���=�����������������
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

