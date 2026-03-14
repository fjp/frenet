function [o_fX, o_fY] = FRT2Cart(i_fS, i_fD, i_faRefRunLength, i_faRefX, i_faRefY)
%FRT2Cart Transform from Frenet s,d coordinates to Cartesian x,y
%   Given Frenet coordinates (i_fS, i_fD) and a reference path defined by
%   cumulative arc lengths (i_faRefRunLength) and points (i_faRefX, i_faRefY),
%   compute the Cartesian position:
%     o_fX - x coordinate in world frame
%     o_fY - y coordinate in world frame

    % Step 1: Find the segment containing arc length s
    nPrevRefPoint = 1;
    while ((nPrevRefPoint < length(i_faRefRunLength)) && (i_fS > i_faRefRunLength(nPrevRefPoint+1)))
        nPrevRefPoint = nPrevRefPoint + 1;
    end
    nNextRefPoint = nPrevRefPoint + 1;

    % Step 2: Compute segment heading
    fHeading = atan2((i_faRefY(nNextRefPoint)-i_faRefY(nPrevRefPoint)), ...
        (i_faRefX(nNextRefPoint)-i_faRefX(nPrevRefPoint)));

    % Step 3: Find the (x,y) point along the segment at arc length s
    % fSegmentS is the remaining arc length within this segment
    fSegmentS = i_fS - i_faRefRunLength(nPrevRefPoint);
    fSegmentX = i_faRefX(nPrevRefPoint) + fSegmentS * cos(fHeading);
    fSegmentY = i_faRefY(nPrevRefPoint) + fSegmentS * sin(fHeading);

    % Step 4: Offset by d in the left-normal direction
    % Rotate heading by +pi/2 (counter-clockwise) to get the left-normal
    % cos(heading + pi/2) = -sin(heading), sin(heading + pi/2) = cos(heading)
    fLeftNormalHeading = fHeading + pi/2;
    o_fX = fSegmentX + i_fD * cos(fLeftNormalHeading);  % = segX - d*sin(heading)
    o_fY = fSegmentY + i_fD * sin(fLeftNormalHeading);  % = segY + d*cos(heading)

end

