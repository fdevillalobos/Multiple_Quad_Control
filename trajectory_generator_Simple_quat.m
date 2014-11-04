function [desired_state] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory

% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments.

% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)

% desired_state: Contains all the information that is passed to the controller

persistent path0 dist acc_time total_time init inde %map0
 
%************************************************************************%
%*                           Input Parameters                           *%
%************************************************************************%
if (nargin > 2)
    map0 = map;
end

%================ Convert Path only to Corner Points ====================%
if (nargin > 3)
    %     path_2 = path{1};
    %     path_dif = diff(path_2);                                                % Get the difference between all points in the path.
    %     for j = 2:size(path_dif,1)
    %         if(min(abs(path_dif(j,:) - path_dif(j-1,:)) <= [0.001 0.001 0.001]) == 1)   % Is next point - actual point == 0, take off point.
    %             path_2(j,:) = [inf inf inf];
    %         end
    %     end
    %
    %     path_2(all(path_2==inf,2),:) = [];                                      % Takes out all invalid points at once.
    %     path0 = path_2;
    %
    %     dist = sqrt(sum(diff(path0).^2,2));                                     % Creates a vector of the absolute distance between points
    %     tot_dist = sum(dist);                                                   % Calculates the total distance to travel
    %     total_time = 20;                                                        % Define time to make the travel
    %     step_time = (dist./tot_dist) * total_time;                              % Calculates the time you would need to do a step according to distance and time
    %     step_time(step_time < 1) = 1;
    %     acc_time = zeros(size(step_time,1)+1,1);
    %     for i = 2:size(step_time)+1
    %         acc_time(i) = acc_time(i-1) + step_time(i-1);                       % Calculates a vector with the accumulated time between points.
    %     end
    %     disp('path 0 Calculated');                                              % Just Checks that it run initialization.
    
    
    max_acc = 2;    % m/s2
    max_vel = 4;    % m/s
    min_t   = max_vel / max_acc;                                            % Ramp up time = Ramp down time
    min_d   = 1/2 * max_acc * min_t^2;                                      % Distance it does while accelerating
    path0 = path{1};                                                        % Maintain path as persistent
    dist = sqrt(sum(diff(path0).^2,2));                                     % Creates a vector of the absolute distance between points
    short_point = dist < (2 * min_d);                                       % Get all the points that are short distances.
    all_other   = (short_point == 0);                                       % All other Points are fine.
    step_time   = zeros(size(dist,1),1);
    step_time(short_point) = sqrt(dist(short_point)/max_acc);
    step_time(all_other)   = 2* min_t + (dist(all_other)-2*min_d)/max_vel;
    
    acc_time = zeros(size(step_time,1)+1,1);
    for i = 2:size(step_time)+1
        acc_time(i) = acc_time(i-1) + step_time(i-1);                       % Calculates a vector with the accumulated time between points.
    end
    fprintf('path 0 Calculated for quadrotor Num %d \n', 1);                % Just Checks that it run initialization.
    
    
    
end


%************************************************************************%
%*                        Initialize Coeficients                        *%
%************************************************************************%
if (nargin >= 3)                                                            % Only goes in here in Initialization
    inde = zeros(6,3,size(path0,1)-1);                                      % Predefines the coeficient matrix for:
                                                                            % X(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
    for i = 2:size(acc_time,1)
        inde(:,:,i-1) = quintic(acc_time(i-1), acc_time(i), path0(i-1,:), path0(i,:));  % Precalculates all the coeficients and stores them in inde
    end
    init = 1;                                                               % Confirms initialization was run.
    pos = path0(end,:)';                                                    % Returns dummy values.
    vel = [0;0;0];
    acc = [0;0;0];
    
%************************************************************************%
%*                        Calculate Pos Vector                          *%
%************************************************************************%
elseif(nargin < 3 && init)                                                  % For every single cycle that is not initialization.
    if (t >= acc_time(end))                                                 % Define final pos for every t > total_time
        pos = path0(end,:)';
        vel = [0;0;0];
        acc = [0;0;0];
    else
        step = t < acc_time;                                                % Defines at which step the copter is.
        act_step = find(step == 0,1,'last');                                % The step is when t < t_next_step
        [pos, vel, acc] = calc_desired(inde(:,:,act_step), t);              % Calculates pos, vel and acc with the corresponding coefs.
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