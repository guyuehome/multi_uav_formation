function [ chosenValley ] = pickValley(valleyArs, T_heading)
% function [ chosenValley ] = pickValley(valleyArs, T_heading)
% pickValley - A function takes in an array of valleys Nx2 and 
% finds which one is closest to the desired T_heading.
%   
% INPUTS:
% valleyArs - a Nx2 array of all candidate valleys.  These are
%             passed as a range.  The first column is the starting index 
%             of the valley, and the second valumn is the ending index. 
%             (can range from 0 to 90)
%
% T_heading - The heading of the target in the range of 1-90.
%
% OUTPUTS:
% chosenValley - The range of the chosen valley which the robot will take
%                given as a 1x2 range.

%Finds the distance to each rising or falling edge
dist = abs(valleyArs-T_heading); 
[r,c] = find(dist==min(dist(:)));
chosenValley = valleyArs(r,:);

%The distance might be the same to two valleys, so we pick the larger one
if size(chosenValley) > 1
    if chosenValley(1) == chosenValley(2)
        chosenValley = unique(chosenValley)';
        %If repeated valley, just take the unique value
    else
        larger = diff(chosenValley');
        [r,t] = max(larger);
        chosenValley = chosenValley(t,:);
        %Picks the larger valley
    end
end
end

