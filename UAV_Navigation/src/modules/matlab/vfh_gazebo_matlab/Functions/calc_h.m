function h=calc_h(m, sector_size)
% function h=calc_h(m, sector_size, current_res)
% calc_h - This function takes a large vector of obstacle
% readings and condenses it to a smaller Polar vector without too much loss
% of the basic obstacle information.
%
% INPUT:
% m = Obstacle field vector with many measurements
% sector_size = The Polar vector size we want to condense to
% 
% OUTPUT:
% h = Reduced Polar vector of obstacle readings

%m(end)=[]; %eliminated for Gazebo
sectors=(length(m))/sector_size;
cnt=0;
for i=1:sectors
    start=(cnt*sector_size)+1;
    stop=sector_size-1;
    h(i)=(sum(m(start:(start+stop))))/sector_size; %Vector of Sectors
    cnt=cnt+1;
end


