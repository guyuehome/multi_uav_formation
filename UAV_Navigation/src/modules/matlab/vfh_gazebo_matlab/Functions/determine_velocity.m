function [v,w]=determine_velocity(chosenheading,hp,Wmax,Vmax)
% function [v,w]=determine_velocity(chosenheading,hp,Wmax,Vmax)
% determine_velocity
% This function calculates the angular and linear velocities of the robot
%
% Inputs:
%   chosenheading: the heading of the robot
%   hp: hp(h') is smoothed polar density in the current direction of travel
%   Wmax: the max allowed angular velocity in rad/s
%   Vmax: the max allowed linear velocity in m/s
%
% Outputs:
%   V: linear velocity (m/s)
%   w: angular velocity (rad/s)
%
% Variables to be set as constants:
%   Vmin: minimum linear velocity of the robot in m/s
%   hm: empirically determined consant that causes a sufficient reduction
%   in speed
%
    % Set the minimum speed of the robot and hm
    Vmin = 0.0; % setting Vmin to 0.0 m/s
    hm = 18;% Set speed reduction constant to 8
    w = -(((chosenheading - 45)*3*2*pi)/60)*Wmax/14.1372;
    %calculate h double prime
    hmid=max(hp(30:60));
    hpp = min(hmid,hm); % guarantees that hpp is less than or equals to hm    
    Vp = Vmax*(1 - hpp/hm);
    v = Vp*(1 - abs(w/Wmax)) + Vmin; %Linear velocity    
end
    
    
    
