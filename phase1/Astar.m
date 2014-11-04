function [path, num_expanded, or_path] = Astar(map, start, goal, draw)
%function [path, num_expanded, or_path] = Astar(map, start, goal, draw)
% Astar Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.
%
%   [PATH, NUM_EXPANDED] = Astar(...) returns the path as well as
%   the number of points that were visited while performing the search.
%
%   [PATH, NUM_EXPANDED] = Astar(...) returns also the original non
%   shortened path that was calculated during Astar
%************************************************************************%
%*                           Input Parameters                           *%
%************************************************************************%
if nargin < 4
    draw = 0;
end

%%
%************************************************************************%
%*                            Initialization                            *%
%*              No need to change anything here for 26 connected        *%
%************************************************************************%

xy_dir = map.param(1);      z_dir  = map.param(2);

sqr_2  = sqrt(2*map.param(1)^2);
sqr_3  = sqrt(2*map.param(1)^2 + map.param(2)^2);
sqr_23 = sqrt(  map.param(1)^2 + map.param(2)^2);

Cmp_Mat(:,:,1) = [  sqr_3   sqr_23  sqr_3;
    sqr_23  z_dir   sqr_23;
    sqr_3   sqr_23  sqr_3];

Cmp_Mat(:,:,2) = [  sqr_2    xy_dir  sqr_2;
    xy_dir   0       xy_dir;
    sqr_2    xy_dir  sqr_2];

Cmp_Mat(:,:,3) = [  sqr_3    sqr_23   sqr_3;
    sqr_23   z_dir    sqr_23;
    sqr_3    sqr_23   sqr_3];


%Converting Start and Goal points to Matrix Dimensions.
pad_value = (size(Cmp_Mat,1)-1)/2;
% All with round before this change. That was what made me pass empty_block.
Start(1) = floor((start(1) - map.boundaries(1)) / map.param(1)) +1 +pad_value;
Start(2) = floor((start(2) - map.boundaries(2)) / map.param(1)) +1 +pad_value;
Start(3) = floor((start(3) - map.boundaries(3)) / map.param(2)) +1 +pad_value;
Goal(1)  = floor((goal(1)  - map.boundaries(1)) / map.param(1)) +1 +pad_value;
Goal(2)  = floor((goal(2)  - map.boundaries(2)) / map.param(1)) +1 +pad_value;
Goal(3)  = floor((goal(3)  - map.boundaries(3)) / map.param(2)) +1 +pad_value;

Vis_Grid = ones(size(map.grid) + 2*pad_value);                              %Creates a padded Visited grid with ones in points that are obstacles or out of bounds.
%Gets all the obstacle positions from map and inserts them on Vis_Grid
Vis_Grid(1+pad_value:pad_value+size(map.grid,1), 1+pad_value:pad_value+size(map.grid,2),1+pad_value:pad_value+size(map.grid,3)) = bsxfun(@ne,map.grid,zeros(size(map.grid)));
Val_Grid = ones(size(map.grid)+ 2*pad_value)*inf;                           %Creates Value grid that has infinites in every point that was not visited.
Val_Grid(Start(1),Start(2),Start(3)) = 0;                                   %Start point will always have cost 0 and its the start point.

if(Vis_Grid(Goal(1),Goal(2),Goal(3)) == 1)                                  %Conversion adjustment for certain values.
    Goal(3)  = ceil((goal(3)  - map.boundaries(3)) / map.param(2)) +1 +pad_value;
end

x_coord = Start(1);   y_coord = Start(2);   z_coord = Start(3);


%% Check that start and goal do not collide.
do_break = 0;
check = collide(map, [start; goal]);
if(sum(check) ~= 0)
    if(check(1) == 1)
        fprintf 'Start Point collides \n'
    end
    if(check(2) == 1)
        fprintf 'Goal Point collides \n'
    end
    do_break = 1;
end

%%
if(do_break == 0)
    while(Vis_Grid(Goal(1), Goal(2), Goal(3)) == 0)
        ActP      = [x_coord, y_coord, z_coord];
        AuxM      = Val_Grid(ActP(1),ActP(2),ActP(3)) + Cmp_Mat;                                    %Matrix that contains distances + value of center.
        Insp_Box  = Val_Grid(ActP(1)-1:ActP(1)+1, ActP(2)-1:ActP(2)+1, ActP(3)-1:ActP(3)+1);        %Inpection Box centered in ActP.
        Not_Vis   = Vis_Grid(ActP(1)-1:ActP(1)+1, ActP(2)-1:ActP(2)+1, ActP(3)-1:ActP(3)+1) == 0;   %Which of those where not visited.
        GRT_val   = Insp_Box > AuxM;                                                                %Which pixels are greater.
        Ins_Block = (Not_Vis & GRT_val).* AuxM;                                                     %Block of values to insert.
        Ins_Block(Ins_Block == 0) = 1;                                      %Replace 0s for 1s.
        Insp_Box(GRT_val & Not_Vis) = 1;                                    %Replace infinities for 1s.
        Val_Grid(ActP(1)-1:ActP(1)+1, ActP(2)-1:ActP(2)+1, ActP(3)-1:ActP(3)+1) = Insp_Box .* Ins_Block;
        Vis_Grid(ActP(1), ActP(2), ActP(3)) = 1;                            %Declare point as already visited.
        Vis_Inf = Val_Grid; Vis_Inf(Vis_Grid == 1) = inf;                   %Creates new matrix that has infinity in all visited values and distances in all the ones that where not visited.
        % Search for next point.
        if(min(Vis_Inf(:)) == inf)
            do_break = 1; break;                                            %Path Blocked, Exit Routine.
        end
        
        %Getting a Matrix of Distances for all values that are not infinity.
        [x_coord, ncol] = find(Vis_Inf ~= inf);                                %Find every index for values that we KNOW distance but havent visited yet.
        y_coord = mod(ncol,size(Vis_Inf,2));
        y_coord(y_coord == 0) = size(Vis_Inf,2);
        z_coord = (ncol-y_coord)/size(Vis_Inf,2) + 1;                       %Convert to x y z coordinates.
        Value_Points = [x_coord y_coord z_coord];
        Vis_InfDist = Vis_Inf;
        Dist_to_Goal = sqrt((Value_Points(:,1)-Goal(1)).^2 + (Value_Points(:,2)-Goal(2)).^2 + (Value_Points(:,3)-Goal(3)).^2);  %Compute Distance to Goal for every Point.
        for i = 1:size(Dist_to_Goal,1);                                     %Get cost to goal for every non visited point.
            Vis_InfDist(Value_Points(i,1), Value_Points(i,2), Value_Points(i,3)) = Vis_InfDist(Value_Points(i,1), Value_Points(i,2), Value_Points(i,3))+ Dist_to_Goal(i);
        end
        Vis_Inf = Vis_Inf + Vis_InfDist;                                    %Add up the cost to get to that point with an approximate of distance to goal.
        
        [~, ind2] = min(Vis_Inf(:));
        [x_coord, y_coord, z_coord] = ind2sub(size(Vis_Inf), ind2);
    end
end
%% First Point of the Path is the Goal
if(do_break == 0)
    Path(1,:) = ActP;   k = 2;
    
    while(Val_Grid(ActP(1), ActP(2), ActP(3)) ~= 0)                         %Trace Back the closer Points to Start.
        Insp_Box = Val_Grid(ActP(1)-1:ActP(1)+1, ActP(2)-1:ActP(2)+1, ActP(3)-1:ActP(3)+1);         %Inpection Box centered in ActP.
        
        [~, ind2] = min(Insp_Box(:));                                       % Finds the minimum value to trace back in the Inspection Box.
        [x_coord, y_coord, z_coord] = ind2sub(size(Insp_Box), ind2);        % Converts those indeces to x y z.
        ActP = [ActP(1)+x_coord-2, ActP(2)+y_coord-2, ActP(3)+z_coord-2];   % Gets new point to trace back.
        Path(k,:) = ActP;                                                   % Adds point to the Path.
        k = k+1;
    end
    Path(:,1) = (Path(:,1) - pad_value - 1) .* map.param(1) + map.boundaries(1);  %Convert Back to Normal Coordinates.
    Path(:,2) = (Path(:,2) - pad_value - 1) .* map.param(1) + map.boundaries(2);
    Path(:,3) = (Path(:,3) - pad_value - 1) .* map.param(2) + map.boundaries(3);
    
    
    %************************************************************************%
    %*                           Result Drawing                             *%
    %************************************************************************%
    if(draw > 0)
        %Initialize Graph Function
        gridX = map.boundaries(1):map.param(1):map.boundaries(4);
        gridY = map.boundaries(2):map.param(1):map.boundaries(5);
        gridZ = map.boundaries(3):map.param(2):map.boundaries(6);
        %tic
        if(draw == 1)
            figure(1);
            cmap = colormap(jet(128));
            grid_2 = map.grid; %Vis_Grid - (map.grid>0);
            grid_2(grid_2 == 0) = 0*inf;
            PATCH_3Darray(grid_2,gridX,gridY,gridZ,'col',cmap); view(3);
            hold on;
            plot3(Path(:,1),Path(:,2),Path(:,3));
            
        elseif(draw == 2)
            figure(2); color_max = ceil(Val_Grid(Goal(1), Goal(2), Goal(3)));
            cmap = colormap(jet(128));
            grid_2 = Val_Grid(1+pad_value:pad_value+size(map.grid,1), 1+pad_value:pad_value+size(map.grid,2),1+pad_value:pad_value+size(map.grid,3));
            grid_2(grid_2 == inf) = 0*inf;
            PATCH_3Darray(grid_2,gridX,gridY,gridZ,cmap,[0 color_max],'col','barw'); view(3);
            
        elseif(draw == 3)
            figure(1); subplot(1,2,1);
            cmap = colormap(jet(128));
            grid_2 = map.grid; %Vis_Grid - (map.grid>0);
            grid_2(grid_2 == 0) = 0*inf;
            PATCH_3Darray(grid_2,gridX,gridY,gridZ,'col',cmap); view(3);
            hold on;
            plot3(Path(:,1),Path(:,2),Path(:,3));
            subplot(1,2,2);
            color_max = ceil(Val_Grid(Goal(1), Goal(2), Goal(3)));
            grid_2 = Val_Grid(1+pad_value:pad_value+size(map.grid,1), 1+pad_value:pad_value+size(map.grid,2),1+pad_value:pad_value+size(map.grid,3));
            grid_2(grid_2 == inf) = 0*inf;
            PATCH_3Darray(grid_2,gridX,gridY,gridZ,cmap,[0 color_max],'col','bars'); view(3);
        end
        set(figure(1),'Units','Normalized','OuterPosition',[0 0 1 1]);
        %toc
    end
    
    %% Results
    path = flipud(Path);                                                    %Flip order so that it starts from START and finished at GOAL
    path(1,:) = start;  path(end,:) = goal;                                 %Force both first and last points to be exactly the start and goal.
    
    %=============== Shorten the Path to Corner Points ==================%
    path_2 = path;
    or_path = path;
    
    %=============== Shorten the Path only to Min Points ================%
    fprintf( 'Adjusting Path to Minimize Points \n');
    for i = 1:size(path,1)-2
        do_break2 = 0;
        for j = i+2:size(path,1)
            if(path_2(i,1) ~= inf && path_2(j,1) ~= inf && size(path_2,2) == 3 && size(path,2) == 3)    % Skip points that have already been declared useless.
                vect = path(j,:) - path(i,:);                           % Vector form one point to the next (skipping at least 1)
                p_num = 2*max(abs(ceil([vect(1)/map.param(1), vect(2)/map.param(1), vect(3)/map.param(2)])));  % Get the number of divisions to be sure it does not collide.
                div_vect = vect/p_num;                                  % Distance to advance each time to evaluate collision.
                colli_vect = zeros(p_num + 1,3);
                for k = 0:p_num                                         % Create collision vector to evaluate new path skipping a point.
                    colli_vect(k+1,:) = k * div_vect + path(i,:);
                end
                colli_vect(end,:) = path_2(j,:);
                A = collide(map, colli_vect);                           % Check for collisions in new trajectory.
                if(max(A) == 0)
                    path_2(j-1,:) = [inf inf inf];                      % Declare point as skippable
                else
                    do_break2 = 1;
                end
            else
                do_break2 = 1;
            end
            
            if(do_break2 == 1)
                break;
            end
        end
    end
    fprintf( 'Shortset Possible Path Found \n');
    path_2(all(path_2==inf,2),:) = [];                                  % Takes out all invalid points at once.
    path = path_2;
    
    %================== If asked for addition Values ====================%
    if nargout>=2
        %         Vis_Grid = Vis_Grid(1+pad_value:pad_value+size(map.grid,1), 1+pad_value:pad_value+size(map.grid,2),1+pad_value:pad_value+size(map.grid,3));
        %         Val_Grid = Val_Grid(1+pad_value:pad_value+size(map.grid,1), 1+pad_value:pad_value+size(map.grid,2),1+pad_value:pad_value+size(map.grid,3));
        num_expanded = sum(sum(sum((Vis_Grid - (map.grid>0)))));
        %varargout(2) = num_expanded;
    end
else
    if(sum(check) == 0)
        fprintf 'Blocked Path \n';
    end
    path = [];
    if nargout>=2
        %         Vis_Grid = Vis_Grid(1+pad_value:pad_value+size(map.grid,1), 1+pad_value:pad_value+size(map.grid,2),1+pad_value:pad_value+size(map.grid,3));
        %         Val_Grid = Val_Grid(1+pad_value:pad_value+size(map.grid,1), 1+pad_value:pad_value+size(map.grid,2),1+pad_value:pad_value+size(map.grid,3));
        num_expanded = sum(sum(sum((Vis_Grid - (map.grid>0)))));
        %varargout(2) = num_expanded;
    end
end

end