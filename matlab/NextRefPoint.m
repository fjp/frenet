function [o_nClosestRefPoint] = NextRefPoint(i_fX, i_fY, i_fPsi, i_faRefX, i_faRefY)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here


    o_nClosestRefPoint = ClosestRefPoint(i_fX, i_fY, i_faRefX, i_faRefY);

    fRefX = i_faRefX(o_nClosestRefPoint);
    fRefY = i_faRefY(o_nClosestRefPoint);

    
    
    fHeading = atan2((i_fY - fRefY), ...
                     (i_fX - fRefX));

    fAngle = abs(i_fPsi - fHeading);
    fAngle = min(2*pi - fAngle, fAngle);

    if(fAngle > pi/4)

        o_nClosestRefPoint = o_nClosestRefPoint - 1;
        if (o_nClosestRefPoint == 0)
            o_nClosestRefPoint = 1;
        
        end
    end
    
    qClosest = quiver(i_faRefX(o_nClosestRefPoint), i_faRefY(o_nClosestRefPoint), i_fX-i_faRefX(o_nClosestRefPoint), i_fY-i_faRefY(o_nClosestRefPoint), 0);

end

