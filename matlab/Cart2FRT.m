function [ o_fS, o_fD ] = Cart2FRT(i_fX, i_fY, i_fPsi, i_faRefX, i_faRefY)
%Cart2FRT Transform from Cartesian x,y coordinates to Frenet s,d coordinates
%   Detailed explanation goes here

%nNextRefPoint = NextRefPoint(i_fX, i_fY, i_fPsi, i_faRefX, i_faRefY);
[nClosestRefPoint, nClosest2ndRefPoint] = ClosestRefPoint(i_fX, i_fY, i_faRefX, i_faRefY);

if (nClosestRefPoint > nClosest2ndRefPoint)
    nNextRefPoint = nClosestRefPoint;
else
    nNextRefPoint = nClosest2ndRefPoint;
end
    

nPrevRefPoint = nNextRefPoint-1;
if(nNextRefPoint == 1)
    
    %nPrevRefPoint  = length(i_faRefX);
    nPrevRefPoint  = 1;
    nNextRefPoint  = 2;
end

fTangentX = i_faRefX(nNextRefPoint) - i_faRefX(nPrevRefPoint);
fTangentY = i_faRefY(nNextRefPoint) - i_faRefY(nPrevRefPoint);

%qTangent = quiver(i_faRefX(nPrevRefPoint), i_faRefY(nPrevRefPoint), fTangentX, fTangentY, 0);

fVecX = i_fX - i_faRefX(nPrevRefPoint);
fVecY = i_fY - i_faRefY(nPrevRefPoint);

%qVec = quiver(i_faRefX(nPrevRefPoint), i_faRefY(nPrevRefPoint), fVecX, fVecY, 0);

% find the projection of vec onto tangential vector
%fProjectedVecNorm = (fVecX*fTangentX + fVecY*fTangentY) / ...
%                  sqrt(fTangentX*fTangentX + fTangentY*fTangentY);
fTangentLength = norm([fTangentX, fTangentY]);
fProjectedVecNorm = dot([fVecX, fVecY], [fTangentX, fTangentY]) / ...
                    fTangentLength;
fProjectedVecX = fProjectedVecNorm * fTangentX/fTangentLength;
fProjectedVecY = fProjectedVecNorm * fTangentY/fTangentLength;

%qProjectedVec = quiver(i_faRefX(nPrevRefPoint), i_faRefY(nPrevRefPoint), fProjectedVecX, fProjectedVecY, 0);

o_fD = Distance(fVecX, fVecY, fProjectedVecX, fProjectedVecY);

% Check if d value is positive or negative using the dot product result
fX1 = i_faRefX(nPrevRefPoint);
fY1 = i_faRefY(nPrevRefPoint);
fX2 = i_faRefX(nNextRefPoint);
fY2 = i_faRefY(nNextRefPoint);

fd = (i_fX - fX1)*(fY2 - fY1)-(i_fY - fY1)*(fX2 - fX1);
nSide = sign(fd);
if (nSide > 0)
    o_fD = o_fD * -1;
end
    

% calculate s value
o_fS = 0;
for i = 1:nPrevRefPoint
    o_fS = o_fS + Distance(i_faRefX(i), i_faRefY(i), i_faRefX(i+1),i_faRefY(i+1));
end

o_fS = o_fS + fProjectedVecNorm;

end

