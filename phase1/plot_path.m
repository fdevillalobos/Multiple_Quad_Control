function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
      %     xmin   ymin       zmin    xmax       ymax     zmax      color
    if nargin < 2
        path = false;
        draw = false;
    else
        draw = true;
    end
    
    for i = 1:size(map.blocks,1)
        block = map.blocks(i,:);
        xmin = block(1);    xmax = block(4);
        ymin = block(2);    ymax = block(5);
        zmin = block(3);    zmax = block(6);
        color = [block(7) block(8) block(9)]/255;
        
        X_1 = [xmin; xmax; xmax; xmin];
        Y_1 = [ymin; ymin; ymax; ymax];
        Z_1 = [zmin; zmin; zmin; zmin];
        
        X_2 = [xmin; xmax; xmax; xmin];
        Y_2 = [ymin; ymin; ymax; ymax];
        Z_2 = [zmax; zmax; zmax; zmax];
        
        X_3 = [xmin; xmin; xmin; xmin];
        Y_3 = [ymin; ymax; ymax; ymin];
        Z_3 = [zmin; zmin; zmax; zmax];
        
        X_4 = [xmax; xmax; xmax; xmax];
        Y_4 = [ymin; ymax; ymax; ymin];
        Z_4 = [zmin; zmin; zmax; zmax];
        
        X_5 = [xmin; xmax; xmax; xmin];
        Y_5 = [ymin; ymin; ymin; ymin];
        Z_5 = [zmin; zmin; zmax; zmax];
        
        X_6 = [xmin; xmax; xmax; xmin];
        Y_6 = [ymax; ymax; ymax; ymax];
        Z_6 = [zmin; zmin; zmax; zmax];
        
        X = [X_1 X_2 X_3 X_4 X_5 X_6];
        Y = [Y_1 Y_2 Y_3 Y_4 Y_5 Y_6];
        Z = [Z_1 Z_2 Z_3 Z_4 Z_5 Z_6];
        
        patch(X,Y,Z,color); view(3); grid on; hold on;        
    end
    xlabel('X(m)');     ylabel('Y(m)');     zlabel('Z(m)');
    
    if(draw == true)
        plot3(path(:,1),path(:,2),path(:,3));
        title('Graphic Representation of Environment and chosen Path');
    else
        title('Graphic Representation of Environment');
    end
end