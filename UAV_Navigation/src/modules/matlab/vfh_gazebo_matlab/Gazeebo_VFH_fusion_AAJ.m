%bingo bingbin.lw@gmail.com
% original author:Adam Fuchs  https://github.com/fuchsat93/Husky_VFH_Gazebo
rosshutdown;
clc;clear;close all
% Initialize ROS 
%setenv("ROS_MASTER_URI","http://tegra-ubuntu:11311");%form tx2 ubuntu
setenv("ROS_MASTER_URI","http://ubuntu:11311");%form vm ubuntu
rosinit;

%lidar=rossubscriber('/scan');%form tx2 ubuntu
lidar=rossubscriber('/lidar2Dscan');%form vm ubuntu

uav_pose=rossubscriber('/mavros/local_position/pose');
robot=rospublisher('/cmd_vel','std_msgs/Float64MultiArray');

%%

% Define global variables agreed upon by our groups
Wmax = 3; %max angular velocity
Vmax = 0.3; %max forward velocity
robotDim = .6; %robot width (m)
threshold = 1; %threshold (m)
sectorSize = 4; %8 lidar sectors per calculation sector
maxRange = 3; %max range in meters
valleyMin = 20; %minimum sectors for a wide valley

% Initialize all data that will be plotted
h=zeros(1,90);
hp=zeros(1,90);
hb=zeros(1,90);
where = 0;
heading = 0;
vel = 0;
angvel = 0;
current = [0 0];
% Update target and "evaluate" target and distance to chose a new heading
target1 = [100 0];
distance1 = 999;


%% - PLOT INITIALIZATION

% A self-updating plot of all important data
VFH_fig = figure(1);
set(VFH_fig,'Position',[50,50,600,650]);
xp=1:90;

% Plots the initial h graph
subplot(4,1,2)
s=plot(xp,h,'YDataSource','h');
axis([1 90 0 8]);
title('h - Danger Detection Input');
text(0,18,'\bfX-POS: ','Color','red','FontSize',12,'BackgroundColor','white');
text(0,16,'\bfY-POS: ','Color','green','FontSize',12,'BackgroundColor','white');
text(0,14,'\bfVEL-X: ','Color','blue','FontSize',12,'BackgroundColor','white');
text(0,12,'\bfVEL-Y: ','Color','black','FontSize',12,'BackgroundColor','white');

t1=text(20,18,num2str(current(1)),'Color','red','FontSize',12,'BackgroundColor','white');
t2=text(20,16,num2str(current(2)),'Color','green','FontSize',12,'BackgroundColor','white');
t3=text(20,14,num2str(vel),'Color','blue','FontSize',12,'BackgroundColor','white');
t4=text(20,12,num2str(angvel),'Color','black','FontSize',12,'BackgroundColor','white');

% Plots initial hp graph - smoothed curve    
subplot(4,1,3)
u=plot(xp,hp,'YDataSource','hp');
title('hp - Averaged Lidar Reading');

% Plots initial hb - threshold graph
subplot(4,1,4)
t=stem(xp,hb,'YDataSource','hb');
title('hb - Data through threshold');
h1=text(heading-1,.7,int2str(heading),'Color','red','FontSize',12);
h2=text(where-1,.2,int2str(where),'Color','blue','FontSize',12,'BackgroundColor','white');

%% 

while distance1 > 0.2
    tic
    
    lidarMsg = lidar.LatestMessage;
    lidarRanges = lidarMsg.Ranges;

    uav_posedata = uav_pose.LatestMessage;
    pose = uav_posedata.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
    z = pose.Position.Z;
    
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = rad2deg(angles(1));
    % Changes theta from a (-180 - 180) to a (0 - 360) for calc_Target
    if theta<0
        theta = theta + 360;
    end

    % Update current position from odometer
    current = [x, y];
    currHeading = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                           VFH ALGORITHM                          %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    m = calcDanger(lidarRanges, maxRange);%ÊýÖµÔœŽóÔœÎ£ÏÕ
    h = calc_h(m, sectorSize);%ŒÆËã90žösectorÖÐÃ¿žöµÄÕÏ°­ÎïÃÜ¶È
    hp = Calc_hp(h,3);%Æœ»¬ŽŠÀí
    hb = calc_Hb(hp, threshold);%¶ÔÃ¿žösectorÊÇ·ñŽæÔÚÕÏ°­ÎïœøÐÐ0Óë1ŽŠÀí
    [valleys,sectToclear] = find_Valleys(hb, threshold, robotDim);
    heading = calc_Target(target1,current,currHeading);
    valley = pickValley(valleys, heading);
    where = pick_Heading(heading,valley,valleyMin,sectToclear);
    V = rosmessage('std_msgs/Float64MultiArray');
    V.Data(1) = Vmax*cos(where*4*pi/180);
    V.Data(2) = Vmax*sin(where*4*pi/180);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                                                  %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Publish calculated forward and angular velocity to Husky
    velmsg=rosmessage(robot);
    velmsg = V;
    send(robot,velmsg);
    
    % Calculate distance for loop checking (see if target is reached)
    distx = target1(1) - current(1);
    disty = target1(2) - current(2);
    dist = sqrt(distx^2 + disty^2);
    distance1 = dist;
    
    % Update plots!
    subplot(4,1,2)
    refreshdata(s,'caller');
    delete(t1); 
    delete(t2); 
    delete(t3); 
    delete(t4);
    t1=text(20,18,num2str(current(1)),'Color','red','FontSize',12,'BackgroundColor','white');
    t2=text(20,16,num2str(current(2)),'Color','green','FontSize',12,'BackgroundColor','white');
    t3=text(20,14,num2str(V.Data(1)),'Color','blue','FontSize',12,'BackgroundColor','white');
    t4=text(20,12,num2str(V.Data(2)),'Color','black','FontSize',12,'BackgroundColor','white');
    drawnow;
    
    subplot(4,1,3)
    refreshdata(u,'caller');
    drawnow;
    
    subplot(4,1,4)
    delete(h1);
    delete(h2);
    hb=double(hb);
    refreshdata(t,'caller');
    h1=text(heading-1,.7,int2str(heading),'Color','red','FontSize',12,'BackgroundColor','white');
    h2=text(where-1,.2,int2str(where),'Color','blue','FontSize',12,'BackgroundColor','white');
    drawnow;
    
    toc
end

    V.Data(1) = 0;
    V.Data(2) = 0;
    velmsg=rosmessage(robot);
    velmsg = V;
    send(robot,velmsg);





