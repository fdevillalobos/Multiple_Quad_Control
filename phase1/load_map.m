function map = load_map(filename, xy_res, z_res, margin, draw)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  
%  Creates an occupancy grid map where a node is considered fill if it
%  lies within 'margin' distance of an obstacle.

if nargin < 5
     draw = 0;
end
%************************************************************************%
%*                      Get Data out of txt document                    *%
%************************************************************************%
%%
fileID = fopen(filename);
comments = textscan(fileID,'%s', 'CommentStyle', '#');
%boundary = textscan(fileID,'%s %f32 %f32 %f32 %f32 %f32 %f32',1);
%blocks = textscan(fileID,'%s %f32 %f32 %f32 %f32 %f32 %f32 %f32 %f32 %f32');
fclose(fileID);
text = comments{1};
l = 0;
boundary = zeros(1,6);
for i = 1:size(text,1)
    if(strcmp(text{i},'boundary') == 1 && size(sscanf(text{i+1},'%f'),1) > 0)
        for j = 1:6
            ch = sscanf(text{i+j},'%f');
            boundary(j) = ch;
        end
    end
    
    if(strcmp(text{i},'block') == 1 && size(sscanf(text{i+1},'%f'),1) > 0)
        l = l + 1;
        for k = 1:9
            ch = sscanf(text{i+k},'%f');
            blocks(l,k) = ch;
        end
    end

end

for i = 1:3
    if(boundary(i) > boundary(i+3))
        aux = boundary(i+3);
        boundary(i+3) = boundary(i);
        boundary(i) = aux;
    end
end

%************************************************************************%
%*                  Define the occupied elements in Space               *%
%************************************************************************%
grid_1 = zeros(ceil((boundary(4)-boundary(1))/xy_res)+1, ceil((boundary(5)-boundary(2))/xy_res)+1, ceil((boundary(6)-boundary(3))/z_res)+1);

if(size(blocks,1)>0)                                                        %Check if there is any block
    %Pad by the Margin pixel size
    if(mod(margin/xy_res,1) - 1.0 <= 0.00000001)
        xy_pad = round((margin/xy_res));
    else
        xy_pad = floor((margin/xy_res));
    end
    if(mod(margin/z_res,1) - 1.0 <= 0.00000001)
        z_pad  = round((margin/z_res));
    else
        z_pad  = floor((margin/z_res));
    end
    
    grid_2 = zeros(size(grid_1,1)+2*xy_pad, size(grid_1,2)+2*xy_pad, size(grid_1,3)+2*z_pad);
    
    %Go Block by Block inserting each padded one.
    for i = 1:size(blocks,1)
        x_max = floor((blocks(i,4) + margin - boundary(1))/xy_res) + 1;    x_min =  ceil((blocks(i,1) - margin - boundary(1))/xy_res) + 1;
        y_max = floor((blocks(i,5) + margin - boundary(2))/xy_res) + 1;    y_min =  ceil((blocks(i,2) - margin - boundary(2))/xy_res) + 1;
        z_max = floor((blocks(i,6) + margin - boundary(3))/z_res)  + 1;    z_min =  ceil((blocks(i,3) - margin - boundary(3))/z_res)  + 1;
        block_grid = ones((x_max-x_min)+1, (y_max-y_min)+1, (z_max-z_min)+1);
        grid_2(x_min+xy_pad:x_max+xy_pad, y_min+xy_pad:y_max+xy_pad, z_min+z_pad:z_max+z_pad) = block_grid;
    end
    
    %Recut the "middle" of the Area, the original one.
    grid_f = grid_2(xy_pad+1:size(grid_1,1)+xy_pad, xy_pad+1:size(grid_1,2)+xy_pad, z_pad+1:size(grid_1,3)+z_pad);
end
%************************************************************************%
%*                           Result Outputs                             *%
%************************************************************************%
map.grid = grid_f;
map.boundaries = boundary;
map.blocks = blocks;
map.param = [xy_res z_res margin];
map.filename = filename;

%************************************************************************%
%*                           Result Drawing                             *%
%************************************************************************%
% tic
if(draw == 1)
    plot_path(map);
elseif(draw == 2)
    gridX = boundary(1):xy_res:boundary(4);
    gridY = boundary(2):xy_res:boundary(5);
    gridZ = boundary(3): z_res:boundary(6);
    cmap = jet(16);
    grid_2 = grid_f;
    grid_2(grid_2 == 0) = 0*inf;
    PATCH_3Darray(grid_2,gridX,gridY,gridZ,'col',cmap); view(3);
end
% toc
end
