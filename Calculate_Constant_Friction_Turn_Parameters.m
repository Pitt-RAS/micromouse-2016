clear
clc
close ALL

% turn tuning parameters
tangential_velocity = .671; % m/s
lookup_table_time_step = .001; %s

% turn parameters
turn_angle = 90; % m/s
ideal_turn_radius = 90; % mm

% robot parameters
mass = 0.15; % kg
moment_of_inertia = 0.0003; % kg-m^2
distance_bt_wheels = 85.5; % mm
friction_coefficient = 0.75; % no unit

gravity = 9.81; % m/s/s
descretizing_time_step = 0.001; % s   This denotes the accuracy of the calculation

% unit conversions
turn_angle = turn_angle * pi / 180; % convert to radians
ideal_turn_radius = ideal_turn_radius / 1000; % convert to m
distance_bt_wheels = distance_bt_wheels / 1000; % convert to m
max_friction_force = friction_coefficient * gravity * mass; % convert friction coefficient into max acceleration

% compute properties of a circular turn
circular_path_distance = turn_angle * ideal_turn_radius; % linear distance traveled by robot in m
circular_path_time = circular_path_distance / tangential_velocity; % total time to complete turn
circular_turn_angular_velocity = tangential_velocity / ideal_turn_radius; % angular velocity throughout turn



% compute constant force turn equations
max_theta_prime = max_friction_force / (mass * tangential_velocity);

syms t
syms theta
syms theta_prime
syms theta_prime_prime
syms inside_trigs
syms abs_sec_func

inside_trigs = t * mass * distance_bt_wheels * tangential_velocity / (2 * moment_of_inertia);
abs_sec_func = sqrt(1/cos(inside_trigs)^2);

theta = (2 * max_friction_force * moment_of_inertia / (mass^2 * tangential_velocity^2 * distance_bt_wheels)) ...
    * (-1 + abs_sec_func) / (abs_sec_func);

theta_prime = (max_friction_force / (mass * tangential_velocity)) * ( ...
    tan(inside_trigs)  - tan(inside_trigs) * (-1 + abs_sec_func) / abs_sec_func);

theta_prime_prime = (max_friction_force * distance_bt_wheels / (2 * moment_of_inertia)) * ( ...
    abs_sec_func^2 - abs_sec_func * (-1 + abs_sec_func) - tan(inside_trigs)^2 ...
    + tan(inside_trigs)^2 * (-1 + abs_sec_func) / abs_sec_func);


%solve(theta_prime, t, 'Real',true)

%accelerate_time = vpasolve(theta_prime - (max_theta_prime-.000001)) 
ezplot(theta, [0,1]);
figure 
ezplot(theta_prime, [0,.1])
figure
ezplot(theta_prime_prime, [0,.1])
%turn_time = vpasolve(-theta - .01)

x_pos = 0;

time = [0:descretizing_time_step:.1];






%discrete_thetas = vpa(subs(theta, 't', time));
discrete_thetas = double(subs(theta, 't', time));

max_theta = max(discrete_thetas);

max_theta_accel = 0;
acceleration_time = 0;
while max_theta_accel ~= max_theta 
    acceleration_time = acceleration_time + descretizing_time_step;
    max_theta_accel = double(subs(theta,'t',acceleration_time));
end

max_theta_prime = double(subs(theta_prime,'t',acceleration_time));
if max_theta_prime < 0 
    acceleration_time = acceleration_time - descretizing_time_step;
    max_theta_prime = double(subs(theta_prime,'t',acceleration_time));
end

max_theta_accel = double(subs(theta,'t',acceleration_time));

if turn_angle / 2 > max_theta_accel
    theta_const = turn_angle - max_theta_accel*2;
else
    display('help, you never exit the acceleration phase')
end 

const_velocity_time = theta_const / max_theta_prime;

xy_coordinate = zeros(1,2);
xy_coordinate_circle = zeros(1,2);
for current_time = descretizing_time_step : descretizing_time_step : acceleration_time*2 + const_velocity_time+.1
    if current_time <= acceleration_time
        xy_addition = tangential_velocity * descretizing_time_step * ...
            [sin(vpa(subs(theta,'t',current_time))) cos(vpa(subs(theta,'t',current_time)))];
    elseif current_time <= acceleration_time + const_velocity_time
        this_theta = max_theta_accel + (current_time - acceleration_time) * max_theta_prime;
        xy_addition = tangential_velocity * descretizing_time_step * ... 
            [sin(this_theta) cos(this_theta)];
    elseif current_time <= acceleration_time*2 + const_velocity_time
        this_theta = (turn_angle - vpa(subs(theta,'t',(acceleration_time*2 + const_velocity_time - current_time))));
        xy_addition = tangential_velocity * descretizing_time_step * ... 
            [sin(this_theta) cos(this_theta)];
     else
         xy_addition = tangential_velocity * descretizing_time_step * ...
             [sin(turn_angle) cos(turn_angle)];
    end
    
    if current_time <= circular_path_time
        this_theta = circular_turn_angular_velocity * current_time;
        [vertical_size_circle, horizontal_size] = size(xy_coordinate_circle);
        xy_coordinate_circle(vertical_size_circle + 1,:) = ideal_turn_radius * [1-cos(this_theta) sin(this_theta)];
    else 
        xy_addition_circle = tangential_velocity * descretizing_time_step * ...
             [sin(turn_angle) cos(turn_angle)];
        [vertical_size_circle, horizontal_size] = size(xy_coordinate_circle);
        xy_coordinate_circle(vertical_size_circle+1,:) = xy_coordinate_circle(vertical_size_circle , :)+xy_addition_circle;
    end
    
    
    % add to xy_coordinate calculations
    [vertical_size, horizontal_size] = size(xy_coordinate);
    xy_coordinate(vertical_size+1,:) = xy_coordinate(vertical_size , :)+xy_addition;
    
end
xy_coordinate

figure
plot(xy_coordinate(:,1),xy_coordinate(:,2))
hold

plot(xy_coordinate_circle(:,1),xy_coordinate_circle(:,2),'r')
axis([.085 .095 0.1 .12])

%const_velocity_radius = tangential_velocity / max_theta_prime;
%const_velocity_path_distance = theta_const *



%circular_path_distance = turn_angle * turn_radius; % linear distance traveled by robot in m
%total_time = circular_path_distance / tangential_velocity; % total time to complete turn
%circular_turn_angular_velocity = tangential_velocity / turn_radius; % angular velocity throughout turn



 









