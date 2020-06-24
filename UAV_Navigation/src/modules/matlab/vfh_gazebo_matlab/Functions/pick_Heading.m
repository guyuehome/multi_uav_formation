function heading=pick_Heading(Th,chosenValley,wideValleyMin,sectToClear)
% function heading=pick_Heading(heading,chosenValley,wideValleyMin)
% pick_Heading - 3/28/16
% This function examines the chosen valley that is passed in and determines
% if it is wide or narrow. Depending on its width, the robot will pick a
% heading that gives it the best path to the target while safely navigating
% the obstacle field.
%
% INPUTS:
% Th = The sector location [1 to 90] of the target in relation to the
%      robots current heading.
% chosenValley = The range of the chosen valley which the robot will take
%                given as a 1x2 range.
% wideValleyMin = Minimum number of sectors to classify a valley as 'WIDE'
% sectToClear = Number of sectors that need to be free for the robot to 
%               pass through without hitting.
%
% OUTPUT:
% heading = Return the sector heading of the robot

buffer=ceil(sectToClear/2)+1; %Buffer so robot does not hug walls to close

%Check to see if the chosen valley is wide or narrow
if diff(chosenValley)<=wideValleyMin
    heading=round(mean(chosenValley));
else
    chosenValley(1)=chosenValley(1)+buffer;
    chosenValley(2)=chosenValley(2)-buffer;
    
    %Direct the Robot towards the portion of the valley that gets it most
    %in line with the target while still avoiding obstacles.
    if Th<chosenValley(1)
        heading=chosenValley(1);
    elseif Th>chosenValley(2);
        heading=chosenValley(2);
    elseif Th>=chosenValley(1) && Th<=chosenValley(2)
        heading=Th;
    end
end