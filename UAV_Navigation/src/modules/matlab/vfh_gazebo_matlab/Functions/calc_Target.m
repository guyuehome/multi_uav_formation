function Th_s=calc_Target(targetPos,currentPos,currentHeading)
% function Th=calc_Target(targetPos,currentPos,currentHeading)
% calc_Target
% This function determines the necessary turn angle that the robot must
% make in order to be in a heading straight on with the robot.
%
% INPUT:
% targetPos = [x,y] coordinates of the target in the Map
% currentPos = [x,y] coordinates of the robot in the Map
% currentHeading = The angle (in degrees)that the front of the robot is
%                  facing from the 0 degree axis *(we set the 0 degree axis
%                  of the robot to be in the EAST heading).
%
% OUTPUT:
% Th_s = The sector location [1 to 90] of the target in relation to the
%        robots current heading.

tX=targetPos(1);    %X coordinate of Target
tY=targetPos(2);    %Y coordinate of Target
cX=currentPos(1);   %X coordiante of Robot
cY=currentPos(2);   %Y coordinate of Robot

dX=tX-cX;   %X difference between the target and the robot
dY=tY-cY;   %Y difference between the target and the robot
theta=abs((atan(dY/dX))*180)/pi; %angle of the obstacle from the EAST or
                                 % WEST axis of the robot. 

%The following loop makes conditional checks to determine the turn angle
%that the robot must make in order to be on a heading straght for the
%target.
if(currentHeading>180)
    currentHeading=currentHeading-360;
end
if dX>0 && dY>=0
    Th=-currentHeading+theta;
    if Th>180
        Th=Th-360;
    end
elseif dX<0 && dY>=0
    Th=-currentHeading+(90+theta);
    if Th>180
        Th=Th-360;
    end
elseif dY<0 && dX<=0
    Th=-180-currentHeading+theta;
    if Th<-180
        Th=Th+360;
    end
elseif dY<0 && dX>=0
    Th=-currentHeading-theta;
    if Th<=-180
        Th=Th+360;
    end
end
if(Th<0)
    Th=360+Th;
end
Th_s=round(Th*90/360); %返回目标点方向所在雷达的sector
                                 %relation to the robots heading.