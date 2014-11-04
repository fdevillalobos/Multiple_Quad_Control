function [C] = collide(map, points, verbose)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector; 
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
if(nargin < 3)
    verbose = false;
end

%Search for Collisions
C = zeros(size(points,1),1);
for i = 1:size(points,1)
    for j = 1:size(map.blocks,1)
        if(points(i,1) <= (map.blocks(j,4) + map.param(3)) && points(i,1) >= (map.blocks(j,1) - map.param(3)))
            if(points(i,2) <= (map.blocks(j,5) + map.param(3)) && points(i,2) >= (map.blocks(j,2) - map.param(3)))
                if(points(i,3) <= (map.blocks(j,6) + map.param(3)) && points(i,3) >= (map.blocks(j,3) - map.param(3)))
                    if(verbose)
                        fprintf( 'Collision detected for point %d with block %d. \n', i,j);
                    end
                    C(i) = 1;
                end
            end
        end
    end
    if(points(i,1) > map.boundaries(4))  %Looks first for out of boundary Points.
        fprintf( 'x dimension of point %d is bigger than boundary(4). x Dim: %d \n',i,points(i,1));
        C(i) = 1;
    elseif(points(i,1) < map.boundaries(1))
        fprintf( 'x dimension of point %d is smaller than boundary(1). x Dim: %d \n',i,points(i,1));
        C(i) = 1;
    elseif(points(i,2) > map.boundaries(5))
        fprintf( 'y dimension of point %d is bigger than boundary(5). y Dim: %d \n', i,points(i,2));
        C(i) = 1;
    elseif(points(i,2) < map.boundaries(2))
        fprintf( 'y dimension of point %d is smaller than boundary(2). y Dim: %d \n',i,points(i,2));
        C(i) = 1;
    elseif(points(i,3) > map.boundaries(6))
        fprintf( 'z dimension of point %d is bigger than boundary(6). z Dim: %d \n', i,points(i,3));
        C(i) = 1;
    elseif(points(i,3) < map.boundaries(3))
        fprintf( 'z dimension of point %d is smaller than boundary(3). z Dim: %d \n',i,points(i,3));
        C(i) = 1;
    end
end
end
