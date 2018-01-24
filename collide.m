function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

C=false(size(points,1),1);

if(isempty(map))
    return;
end


for m=1:size(points,1)

    for b=2:size(map,1)-1 %2 since the first row is boundary,-1 since the last row is resolutions
        
         if (points(m,1)>=map(b,1) && points(m,1)<=map(b,4)) &&...
                  (points(m,2)>=map(b,2) && points(m,2)<=map(b,5)) &&...
                  (points(m,3)>=map(b,3) && points(m,3)<=map(b,6))
            C(m)=1;
            break;
         end 
    end
    
    if ~(points(m,1)>=map(1,1) && points(m,1)<=map(1,4) &&...
            points(m,2)>=map(1,2) && points(m,2)<=map(1,5) &&...
            points(m,3)>=map(1,3) && points(m,3)<=map(1,6))
        C(m)=1;
    end
end
        

end