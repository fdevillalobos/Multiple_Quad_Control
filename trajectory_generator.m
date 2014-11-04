function [desired_state, acc_time1, path01] = trajectory_generator(t, qn, qnpos, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory

% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments.

% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)

% desired_state: Contains all the information that is passed to the controller

persistent path0 acc_time init inde qpos nquad %map0 total_time dist copter_distance
 
%************************************************************************%
%*                           Input Parameters                           *%
%************************************************************************%

%% ================ Time Calculation betwenn Points =====================%
if (nargin > 4)
    %%
    % map0 = map;
    if(size(qn,1) == 0)
        qn = size(path,2);
        nquad = qn;
        qpos = zeros(nquad,3);
        %copter_distance = zeros(nquad, nquad);
    end
    max_path = 0;
    for i = 1:qn
        if(size(path{i},1) > max_path)                                      %Adjusts all sizes of paths to the same
            max_path = size(path{i},1);
        end
    end
    
    max_acc = 0.8;    % m/s2
    max_vel = 3.5;    % m/s
    min_t   = max_vel / max_acc;                                            % Ramp up time = Ramp down time
    min_d   = 1/2 * max_acc * min_t^2;                                      % Distance it does while accelerating
    acc_time = zeros(size(path{1},1),qn);
    inde = zeros(6,3,size(path{1},1)-1,qn);                                 % Predefines the coeficient matrix.
    path0 = zeros(max_path,3,qn);
    
    for j = 1:qn
        path0(1:size(path{j},1),:,j) = path{j};                             % Maintain path as persistent
        path0(size(path{j},1)+1:end,:,j) = repmat(path{j}(end,:),max_path-size(path{j},1),1);
        dist = sqrt(sum(diff(path0(:,:,j)).^2,2));                          % Creates a vector of the absolute distance between points
        short_point = dist < (2 * min_d);                                   % Get all the points that are short distances.
        all_other   = (short_point == 0);                                   % All other Points are fine.
        step_time   = zeros(size(dist,1),1);
        step_time(short_point) = sqrt(dist(short_point)/max_acc);
        step_time(all_other)   = 2* min_t + (dist(all_other)-2*min_d)/max_vel;
        
        for i = 2:size(step_time)+1
            acc_time(i,j) = acc_time(i-1,j) + step_time(i-1);               % Calculates a vector with the accumulated time between points.
        end
        fprintf('path 0 Calculated for quadrotor Num %d \n', j);            % Just Checks that it run initialization.
        %************************************************************************%
        %*                        Initialize Coeficients                        *%
        %************************************************************************%
        %if (nargin >= 3)                                                   % Only goes in here in Initialization
        for i = 2:size(acc_time(:,j),1)
            inde(:,:,i-1,j) = quintic(acc_time(i-1,j), acc_time(i,j), path0(i-1,:,j), path0(i,:,j));  % Precalculates all the coeficients and stores them in inde
        end
        init = 1;                                                           % Confirms initialization was run.
        pos = path0(end,:,j)';                                              % Returns dummy values.
        vel = [0;0;0];
        acc = [0;0;0];
    end
    if(nargout > 1)
        acc_time1 = acc_time;
        path01 = path0;
    end
end

if (nargin > 2 && size(qnpos,1) == 3)
    qpos(qn,:) = qnpos(1:3)';
end
%%  
%************************************************************************%
%*                        Calculate Pos Vector                          *%
%************************************************************************%
if(nargin < 3 && init)                                                      % For every single cycle that is not initialization.
    if (t >= acc_time(end,qn))                                              % Define final pos for every t > total_time
        pos = path0(end,:,qn)';
        vel = [0;0;0];
        acc = [0;0;0];
    else
%         if(t > 0.1)
%             for i = 1:nquad
%                 copter_distance(qn,i) = sqrt(sum((s{qn}(1:3)-s{i}(1:3)).^2));
%                 copter_distance
%             end
%         end
        step = t < acc_time(:,qn);                                      % Defines at which step the copter is.
        act_step = find(step == 0,1,'last');                            % The step is when t < t_next_step
        [pos, vel, acc] = calc_desired(inde(:,:,act_step,qn), t);       % Calculates pos, vel and acc with the corresponding coefs.
    end

else
    if(init == 0)
        disp('Coeficients Have not Been Initializated');
    end
    pos = [0;0;0];
    vel = pos;
    acc = pos;
end

%************************************************************************%
%*                          Output Parameters                           *%
%************************************************************************%
yaw = 0;
yawdot = 0;
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
end



%************************************************************************%
%*                          Auxiliary Functions                         *%
%************************************************************************%
function x = quintic(t0, tf, Param0, Param1)                                % Quintic trayectory calculation
   
    A = [1  t0  t0^2    t0^3     t0^4      t0^5;                            % B = A*x
         0   1  2*t0  3*t0^2   4*t0^3    5*t0^4;                            % B has the pos, vel and acc for start and stop points.
         0   0   2    6*t0    12*t0^2   20*t0^3;                            % A is always the same as defined here
         1  tf  tf^2    tf^3     tf^4      tf^5;                            % x has all the necesary coeficients for the quintic.
         0   1  2*tf  3*tf^2   4*tf^3    5*tf^4;
         0   0   2    6*tf    12*tf^2   20*tf^3];
   
    vel0    = [0,0,0];
    acc0    = [0,0,0];
    B = [Param0; vel0; acc0; Param1; vel0; acc0];
   
    x = A\B;
   
end

function [pos, vel, acc] = calc_desired(x, t)                               % With the desired coeficients, it calculates the pos, vel and acc for time t.

   pos = (x(1,:) + x(2,:)*t + x(3,:)*t^2 + x(4,:)*t^3 + x(5,:)*t^4 + x(6,:)*t^5)';
   vel =      (x(2,:) + 2*x(3,:)*t + 3*x(4,:)*t^2 + 4*x(5,:)*t^3 + 5*x(6,:)*t^4)';
   acc =                 (2*x(3,:) + 6*x(4,:)*t + 12*x(5,:)*t^2 + 20*x(6,:)*t^3)';

end