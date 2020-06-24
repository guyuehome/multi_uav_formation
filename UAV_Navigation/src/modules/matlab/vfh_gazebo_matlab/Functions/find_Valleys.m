function [passable,sectToClear]=find_Valleys(Hb, threshold, robotW)
% find_Valleys(Hb, threshold, robotW)
% find_Valleys - Function which takes in a
% 1x90 Array of sectors and decides which sectors can be classified as wide
% enough valleys to travel through.
%
% INPUTS:
% Hb = 1x90 Polar array of binary values(1=obstacle, 0=no obstacle)
% threshold = preselected paramater of minimum obstacle clearance (meters)
% robotW = width of the robot (meters)
%
% OUTPUTS:
% passable = Array of accessable valleys avaialable to the robot.
% sectToClear = Number of sectors that need to be free for the robot to 
%               pass through without hitting.

%determine the amount of consecutive sectors that must be clear in order
%for robot to safely navigate between obstacles.
clearanceAngle=(2*(asin(robotW/(2*threshold)))*180/pi); %degrees 2*asin(0.6/2*1)*180/pi = 35度
sectToClear=floor(clearanceAngle/3); %Number of sectors that need to be free
                                    %for the robot to pass through without
                                    %hitting.
                                    
b1=(find(diff(Hb)==-1)); %Find beginning of consecutive clear sectors 寻找安全谷起始sector
b1=b1+1;
b2=(find(diff(Hb)==1)); %Find end of consecutive clear sector 寻找安全谷结束sector

if (Hb(1) == 0)
    b1=[0,b1];
end
if (Hb(90) == 0)
    b2=[b2,90];
end

passable = [b1;b2];
clear=(diff(passable))+1;%安全谷大小
clear=clear>=sectToClear;
passable(:,(clear==0))=[]; %Eliminate consecutive clear sectors that are
                           %not wide endough for the robot to navigate
                           %through.
passable = passable';