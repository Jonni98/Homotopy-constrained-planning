clear
clc

% startQ = [pi/2 pi/4 pi/2];
% startQ = [pi/2 pi/4 pi/2 pi/4 pi/2];
% startQ = [pi/2 pi/2 pi/2 pi/2 pi/2];
startQ = [pi/4 pi/2 pi/2 pi/2 pi/4];
% goalQ = [pi/2 pi/2 pi/2 pi/2 pi/2];
% goalQ = [pi/8 3*pi/4 pi];
goalQ = [pi/9 3*pi/4 pi 0.9*pi 1.5*pi];
% goalQ = [pi/4 0 pi/2 pi/4 pi/2];

% Shohin's examples
% startQ = [1.05086 2.29946 4.63818 0.898633 2.46497];
% goalQ = [0.141233 1.25016 0.0649993 3.30138 3.62302];
% startQ = [1.61411 0.640882 3.54232 6.25819 3.26249];
% goalQ = [1.40824 2.18801 0.963804 5.08073 2.00193];


runtest('map4.txt', startQ, goalQ, 2)
% runtest('map2.txt', startQ, goalQ, 0)
