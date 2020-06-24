function m = calcDanger(Lidar,maxRange)
% function m = calcDanger(Lidar,maxRange)
% calcDanger
% This function determines the magnitude of the obstacle presence based on
% its proximity to the robot.
%
% INPUTS:
% Lidar = Vector of distance readings
% maxRange = The max distance at which we are concerned about obstacles
%
% OUTPUTS: 
% m = obstacle magnitude vector 

d=Lidar; % Input of the Lidar readings
a=maxRange;
b=1;
% if any Lidar readings exceed the maxRange value + 0.5 m will return 0 I
% beleive this to be true because since the new m value will not detect
% obsticles we are concerned with, thus indicating there is a valley
% inbetween obstibles within a range we care about
for i = 1:length(d)
    if d(i) >= maxRange + 0.5
        m(i) = 0;
    else
        m(i)=a-d(i)*b;
    end
end
end
% how do we incorporate a and b into the calculation
% we want d=a/b