function [ o_fS, o_fD ] = Cart2FRT(i_fX, i_fY, i_faRefX, i_faRefY)
%Cart2FRT Transform from Cartesian x,y coordinates to Frenet s,d coordinates
%   Given a query point (i_fX, i_fY) and a reference path defined by
%   (i_faRefX, i_faRefY), compute the Frenet coordinates:
%     o_fS - arc length along the reference path to the projection point
%     o_fD - signed lateral offset (positive = left of path direction)

% Step 1: Find the two closest reference points and determine the segment
[nClosestRefPoint, nClosest2ndRefPoint] = ClosestRefPoint(i_fX, i_fY, i_faRefX, i_faRefY);

% The "next" ref point is the one further along the path
if (nClosestRefPoint > nClosest2ndRefPoint)
    nNextRefPoint = nClosestRefPoint;
else
    nNextRefPoint = nClosest2ndRefPoint;
end

nPrevRefPoint = nNextRefPoint-1;
if(nNextRefPoint == 1)
    nPrevRefPoint  = 1;
    nNextRefPoint  = 2;
end

% Step 2: Compute the tangent vector of the segment
fTangentX = i_faRefX(nNextRefPoint) - i_faRefX(nPrevRefPoint);
fTangentY = i_faRefY(nNextRefPoint) - i_faRefY(nPrevRefPoint);

%qTangent = quiver(i_faRefX(nPrevRefPoint), i_faRefY(nPrevRefPoint), fTangentX, fTangentY, 0);

% Step 3: Compute vector from previous ref point to query point
fVecX = i_fX - i_faRefX(nPrevRefPoint);
fVecY = i_fY - i_faRefY(nPrevRefPoint);

%qVec = quiver(i_faRefX(nPrevRefPoint), i_faRefY(nPrevRefPoint), fVecX, fVecY, 0);

% Step 4: Project query vector onto the tangent to find the projection point
fTangentLength = norm([fTangentX, fTangentY]);
fProjectedVecNorm = dot([fVecX, fVecY], [fTangentX, fTangentY]) / ...
                    fTangentLength;
fProjectedVecX = fProjectedVecNorm * fTangentX/fTangentLength;
fProjectedVecY = fProjectedVecNorm * fTangentY/fTangentLength;

%qProjectedVec = quiver(i_faRefX(nPrevRefPoint), i_faRefY(nPrevRefPoint), fProjectedVecX, fProjectedVecY, 0);

% Step 5: Compute lateral offset d (distance from query to projection)
o_fD = Distance(fVecX, fVecY, fProjectedVecX, fProjectedVecY);

% Determine sign of d using cross product: positive = left of path
fX1 = i_faRefX(nPrevRefPoint);
fY1 = i_faRefY(nPrevRefPoint);
fX2 = i_faRefX(nNextRefPoint);
fY2 = i_faRefY(nNextRefPoint);

fd = (i_fX - fX1)*(fY2 - fY1)-(i_fY - fY1)*(fX2 - fX1);
nSide = sign(fd);
if (nSide > 0)
    o_fD = o_fD * -1;
end

% Step 6: Compute arc length s by summing segment lengths up to
% nPrevRefPoint, then adding the projected distance along the segment
o_fS = 0;
for i = 1:nPrevRefPoint-1
    o_fS = o_fS + Distance(i_faRefX(i), i_faRefY(i), i_faRefX(i+1),i_faRefY(i+1));
end

o_fS = o_fS + fProjectedVecNorm;

end

