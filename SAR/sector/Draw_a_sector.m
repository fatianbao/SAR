function [  sector ] = Draw_a_sector( map, center,StartR, EndR, StartAngle, EndAngle )

%% Get indexs(row,column)
size_map=size(map);
for i = 1:size_map(2)
    indexs(:,i,1) = 1:size_map(1);%number of row
end
for i = 1:size_map(1)
indexs(i,:,2) = 1:size_map(2);%number of column
end
%% Get radials and angle according to its row and column
radius = sqrt((indexs(:,:,1)-center(1)).^2+(indexs(:,:,2)-center(2)).^2);
indexs(find(indexs(:,:,1) == center(1))) = indexs(find(indexs(:,:,1) == center(1))) + 0.0001;
angle = atan((indexs(:,:,1)-center(1))./(indexs(:,:,2)-center(2)));
angle(find(indexs(:,:,2) < center(2))) = angle(find(indexs(:,:,2) < center(2))) + pi;
angle(find(angle < 0)) = angle(find(angle < 0)) + 2*pi;
 %% get sector
 sector_r = zeros(size_map(1),size_map(2));
 sector_r(find(radius>=StartR & radius <= EndR)) = 1;
 EndA = 2*pi - StartAngle;
 StartA = 2*pi -EndAngle;
 sector_a = zeros(size_map(1),size_map(2));
 sector_a(find(angle<=EndA & angle>=StartA)) = 1;
 sector = sector_r & sector_a;
end