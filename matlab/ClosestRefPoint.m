function [o_nClosestRefPoint, o_nClosest2ndRefPoint] = ClosestRefPoint(i_fX, i_fY, i_faRefX, i_faRefY)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


fClosestLen = inf;
o_nClosestRefPoint = 1;

for i = 1:length(i_faRefX)
    fRefX = i_faRefX(i);
    fRefY = i_faRefY(i);
    
    fDist = Distance(i_fX, i_fY, fRefX, fRefY);
    if(fDist < fClosestLen)
        
        fClosestLen = fDist;
        o_nClosestRefPoint = i;
    else
        break;
    end 
end


fRefXp1 = i_faRefX(o_nClosestRefPoint+1);
fRefYp1 = i_faRefY(o_nClosestRefPoint+1);
fDistp1 = Distance(i_fX, i_fY, fRefXp1, fRefYp1);

fRefXm1 = i_faRefX(o_nClosestRefPoint-1);
fRefYm1 = i_faRefY(o_nClosestRefPoint-1);
fDistm1 = Distance(i_fX, i_fY, fRefXm1, fRefYm1);

if(fDistm1 < fDistp1)
    o_nClosest2ndRefPoint = o_nClosestRefPoint - 1;
else
    o_nClosest2ndRefPoint = o_nClosestRefPoint + 1;
end

end

