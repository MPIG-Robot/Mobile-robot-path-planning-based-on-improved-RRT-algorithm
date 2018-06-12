%% distanceCost.m
function h=distanceCost(a,b)
% 	h = sqrt(sum((a-b).^2, 2));
h = sqrt((a(:,1)-b(:,1)).^2 + (a(:,2)-b(:,2)).^2 );