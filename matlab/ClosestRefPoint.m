function [o_nClosestRefPoint, o_nClosest2ndRefPoint] = ClosestRefPoint(i_fX, i_fY, i_faRefX, i_faRefY)
%ClosestRefPoint Find the two closest reference path points to a query point

[~, SortIdx] = sort(vecnorm([i_fX - i_faRefX; i_fY - i_faRefY]));
o_nClosestRefPoint = SortIdx(1);
o_nClosest2ndRefPoint = SortIdx(2);

end

