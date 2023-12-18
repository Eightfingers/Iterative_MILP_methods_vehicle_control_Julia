using JuMP
using HiGHS
using Plots

mutable struct StateVector
    x::Float64
    y::Float64
    x_vel::Float64
    y_vel::Float64
    x_accel::Float64
    y_accel::Float64
end

mutable struct CircularObstacle
    x::Float64
    y::Float64
    radius::Float64
end

function isCollision(point1, point2, obstacle_center, obstacle_radius)
    # Copied from https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
    # with an additional check at the front to see if both points are inside a circle
    # variables are kept this way for readability
    ax = point1[1]
    ay = point1[2]
    bx = point2[1]
    by = point2[2]
    cx = obstacle_center[1]
    cy = obstacle_center[2]
    r = obstacle_radius

    # Check if both points are inside a circle
    function inCircle(cx,cy, px,py, radius)
        dist = sqrt((cx - px) ^ 2 + (cy - py) ^ 2)
        return dist <= radius
    end

    if inCircle(cx, cy, ax, ay, r) && inCircle(cx, cy, bx, by, r)
        return true
    end

    ax -= cx;
    ay -= cy;
    bx -= cx;
    by -= cy;
    a = (bx - ax)^2 + (by - ay)^2;
    b = 2*(ax*(bx - ax) + ay*(by - ay));
    c = ax^2 + ay^2 - r^2;
    disc = b^2 - 4*a*c;
    if(disc <= 0) 
        return false;
    end
    sqrtdisc = sqrt(disc);
    t1 = (-b + sqrtdisc)/(2*a);
    t2 = (-b - sqrtdisc)/(2*a);

    if ( (0 < t1 && t1 < 1) || (0 < t2 && t2 < 1) ) 
        return true;
    end

    return false;
end

function generatePolygon(k, x, y, radius)
    polygon_x_coordinates = zeros(Float64, k)  # Preallocate arrays
    polygon_y_coordinates = zeros(Float64, k)
    for i in 1:k 
        polygon_x_coordinates[i] = x + radius*sin(2*pi*i/k)
        polygon_y_coordinates[i] = y + radius*cos(2*pi*i/k)
    end
    return polygon_x_coordinates, polygon_y_coordinates
end

function checkCollision(solution, _obstacle)
    collision_indexes = []
    for i in 1:(length(solution)-1)
        point1 = [solution[i].x , solution[i].y]
        point2 = [solution[i+1].x , solution[i+1].y]
        obstacle_center = [_obstacle.x, _obstacle.y]
        println("Checking index:", i )
        println("Point1: ", point1[1], "," , point1[2] )
        println("Point2: ", point2[1], "," , point2[2] )
    
        if isCollision(point1, point2, obstacle_center, _obstacle.radius)
            println("Collision at time index ",i, " and ", i+1)
            display(plot!([solution[i].x, solution[i+1].x] , [solution[i].y, solution[i+1].y], seriestype=:scatter, aspect_ratio=:equal, color=:red))
            push!(collision_indexes, [i,i+1])
        end
    end
    return collision_indexes
end

function computeTimeIntervals(_collision_indexes, time_intervals)
    if length(_collision_indexes) != 0
        for chord in _collision_indexes
            idx = chord[1]
            # println("Adding to collision index ..", idx)
            new_time_interval = time_intervals[idx] / 2
            # println("New time interval ..", new_time_interval)
            # Insert the time interval
            time_intervals[idx] = new_time_interval
            time_intervals = [time_intervals[1:idx]; new_time_interval; time_intervals[idx+1:end]]
            
        end
        # Print the updated vecto
        # println("New time intervals vector: ", new_time_interval)
        println(time_intervals)
        return time_intervals
    else
        return time_intervals
    end 
end

