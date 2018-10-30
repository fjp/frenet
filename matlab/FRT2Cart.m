function [o_fX, o_fY] = FRT2Cart(i_fS, i_fD, i_faRefRunLength, i_faRefX, i_faRefY)
%FRT2Cart Transform from Frenet s,d coordinates to Cartesian x,y
%   Detailed explanation goes here

    nNextRefPoint = 0;

    while ((i_fS > i_faRefRunLength(nNextRefPoint+1)) && (nNextRefPoint < length(i_faRefRunLength)))

        nNextRefPoint = nNextRefPoint + 1;
    end

    nPrevRefPoint = nNextRefPoint-1;

    fHeading = atan2((i_faRefY(nNextRefPoint)-i_faRefY(nPrevRefPoint)), ...
        (i_faRefX(nNextRefPoint)-i_faRefX(nPrevRefPoint)));
    % The x,y,s along the segment
    fSegmentS = i_fS - i_faRefRunLength(nPrevRefPoint);

    fSegmentX = i_faRefX(nPrevRefPoint) + fSegmentS * cos(fHeading);
    fSegmentY = i_faRefY(nPrevRefPoint) + fSegmentS * sin(fHeading);

    fPerpendicularHeading = fHeading - pi/2;

    o_fX = fSegmentX + i_fD * cos(fPerpendicularHeading);
    o_fY = fSegmentY - i_fD * sin(fPerpendicularHeading);

end

