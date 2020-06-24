function Hbin = calc_Hb(hp, threshold)
% function Hbin = Calc_Hb(hp, threshold)
% Calc_Hb
% Generates a binary array (Hbin) which signifies when a value of h' 
% exceeds a given threshold
%
% INPUTS:
% hp = smoothed Sector Vector of obstacles
% threshold = distance at which we want to detect obstacles for the robot
%             to react properly.
%
% OUTPUT:
% Hbin = Binary Vector showing where obstacles and valleys are in relation
%        to the robots lidar view

Hbin = gt(hp,threshold);%if hp(i)>threshold -> Hbin(i)=1,
                          %otherwise Hbin(i)=0