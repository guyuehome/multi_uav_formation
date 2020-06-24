function hp = Calc_hp(h, L)
% function hp = Calc_hp(h, L)
% Calc_hp
% This function will act as a smoothing function for the sector values
% Given h and the lenght of a sector, it will calculate a new array with
% the averaged values.
%
% INPUTS:
% h = Sector Vector of Obstacle readings
% L = Width of filter
%
% OUTPUT:
% hp = Filtered Sector Vector with smoothed out obstacle edges

h_length = length(h);       %get length of sector array
h_padded = [zeros(1,L),h,zeros(1,L)];   %padding h with zeros on the ends to make average calculations
hp_sum = zeros(1,h_length); %initialize array for summation of sector values
div = ones(1,h_length);     %initialize divisor for each element
weightArray = [1:L,L:-1:1];  %weighted array to determine coefficients for avg calc

%Since the first & last L indicies will not have 2L+1 values to divide by
%the divisor must be modified for the end indicies. 
div = div*(2*L+1);          %divisor for calculating avg sector value
div_mod = [L:-1:1, zeros(1,h_length-(2*L)), 1:L];    %modifier array
div = div-div_mod;          %accounting for the missing end values

%loop to calculate the sum of the weighted average for each sector
for i=1+L:h_length+L
    hp_sum(i-L) = sum(h_padded((i-L):(i+L-1)).*weightArray);    %sector summing array
end
hp = hp_sum./div;       %calculate h'


    
