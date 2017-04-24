% Compute neighbors
function [neighbors] = computeNeighbors(node,currNode,prevt,MSN,p)
    neighbors = [];
    for neighbor = 1:p.maxnodes
        if node ~= neighbor
            currNeighbor = [MSN.pos(prevt,neighbor,1) MSN.pos(prevt,neighbor,2) MSN.pos(prevt,neighbor,3)];
            distance = norm(currNode - currNeighbor,2);
            distance = sqrt((currNeighbor(1) - currNode(1))^2 + ...
                (currNeighbor(2) - currNode(2))^2 + ...
                (currNeighbor(3) - currNode(3))^2);
            if distance < p.r
                neighbors(end+1)=neighbor;
   %             A(node,neighbor) = 1;
            end
        end
    end
return