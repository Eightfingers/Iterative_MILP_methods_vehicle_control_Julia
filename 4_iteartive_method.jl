include("helpers.jl")
function solveIterativeVehicleMilp(_initial_state::StateVector, _final_state::StateVector, _discretization_steps, _obstacle, _time_intervals, _collision_indexes)
    model = Model(HiGHS.Optimizer)
    k = _discretization_steps  # Discrete amount
    time_intervals = _time_intervals
    M = 1000
    # Now add length collision indices into k and modify time_intervals accordingly
    
    poly_d = 8  # Polygon discretization amount
    absolute_polygon_x_constraint, absolute_polygon_y_constraint = generatePolygon(poly_d, 0, 0, 1)
        
    #Variables set up
    @variable(model, t[1:k] >=0)
    @variable(model, x[1:k]) # X coordinate
    @variable(model, y[1:k]) # Y coordinate
    @variable(model, x_vel[1:k]) # X velocity
    @variable(model, y_vel[1:k]) # Y velocity
    @variable(model, x_accel[1:k]) # X acceleration
    @variable(model, y_accel[1:k]) # Y acceleration
    @variable(model, z_ax[1:k]) # X accel
    @variable(model, z_ay[1:k]) # Y accel
    @variable(model, z_avoid[1:k, 1:poly_d], Bin) # Obstacle avoidance
    @variable(model, z_reached[1:k], Bin) # Minimum time formulation
    
    # Minimal effort
    @objective(model, Min, sum(z_ax[i] for i in 1:(k)) + sum(z_ay[i] for i in 1:(k)))

    # Minimum Time
    # @objective(model, Min, sum(z_reached[i] * time_intervals[i] for i in 1:(k-1)))
    
    # Absolute values constraint
    @constraints(model, begin z_ax >= x_accel; z_ax >= -x_accel end)
    @constraints(model, begin z_ay >= y_accel; z_ay >= -y_accel end)
    
    # Initial Vehicle Conditions 
    @constraint(model, x[1] == _initial_state.x )
    @constraint(model, y[1] == _initial_state.y )
    @constraint(model, x_vel[1] == _initial_state.x_vel )
    @constraint(model, y_vel[1] == _initial_state.y_vel )
    @constraint(model, x_accel[1] == _initial_state.x_accel )
    @constraint(model, y_accel[1] == _initial_state.y_accel )
    
    # Final vehicle Conditions
    @constraint(model, x[k] == _final_state.x)
    @constraint(model, y[k] == _final_state.y)
    @constraint(model, x_vel[k] == _final_state.x_vel)
    @constraint(model, y_vel[k] == _final_state.y_vel)
    
    # Speed / Accel constraint
    threshold = 2.5e-10
    absolute_polygon_x_constraint[abs.(absolute_polygon_x_constraint) .< threshold] .= 0.0
    absolute_polygon_y_constraint[abs.(absolute_polygon_y_constraint) .< threshold] .= 0.0
    for i in 1:k
        for j in 1:poly_d
            @constraint(model, (absolute_polygon_x_constraint[j] * x_vel[i] + absolute_polygon_y_constraint[j] * y_vel[i]) <= speed_max )
            @constraint(model, (absolute_polygon_x_constraint[j] * x_accel[i] + absolute_polygon_y_constraint[j] * y_accel[i]) <= accel_max )
        end
    end
    
    # Dynamics constraints
    for i in 1:k-1
        @constraint(model, x[i+1] == x[i] + x_vel[i] * time_intervals[i])
        @constraint(model, y[i+1] == y[i] + y_vel[i] * time_intervals[i])
        @constraint(model, x_vel[i+1] == x_vel[i] + (x_accel[i] * time_intervals[i]))
        @constraint(model, y_vel[i+1] == y_vel[i] + (y_accel[i] * time_intervals[i]))
    end 
    
    # Obstacle constraints
    for i in 1:k
        for j in 1:poly_d
            @constraint(model, (x[i] .- _obstacle.x) .* sin(2*pi*j/poly_d) + (y[i] .- _obstacle.y) .* cos(2*pi*j/poly_d) >= _obstacle.radius .- M*z_avoid[i,j])
        end
    end
    @constraint(model, [j=1:k], sum(z_avoid[j, k] for k in 1:8) <= poly_d - 1)
    
    # Minimum Time constraint
    # @constraint(model, [i=1:k], x[i] .- _final_state.x <= M * (z_reached[i]))
    # @constraint(model, [i=1:k], x[i] .- _final_state.x >= - M * (z_reached[i]))
    # @constraint(model, [i=1:k], y[i] .- _final_state.y <= M * (z_reached[i]))
    # @constraint(model, [i=1:k], y[i] .- _final_state.y >= - M * (z_reached[i]))
    # @constraint(model, sum(z_reached[i] for i in k) == 1)
    
    # Solve the MILP
    optimize!(model)
    
    # Retrieve the solution
    x_sol = value.(x)
    y_sol = value.(y)
    x_vel = value.(x_vel)
    y_vel = value.(y_vel)
    x_accel = value.(x_accel)
    y_accel = value.(y_accel)
    z_val = value.(z_avoid)
    
    for i in 1:k
        println("*********** ", i, " ***********")
        println("X: ", x_sol[i], " Y: ", y_sol[i])
        println("x_vel: ", x_vel[i], " y_vel: ", y_vel[i])
        println("x_accel: ", x_accel[i], " y_accel: ", y_accel[i])
    end
    
    accel_sum = 0
    for i in 1:k 
        accel_sum += abs(x_accel[i]) + abs(y_accel[i])
    end
    
    time_sum = 0
    for i in 1:(k-1)
        time_sum += value.(time_intervals[i]) * value.(z_reached[i])
    end
    
    println("Total effort ", accel_sum)
    println("Time taken total ", time_sum)
    
    solution_vectors = StateVector[]
    for i in 1:k
        solution = StateVector(x_sol[i], y_sol[i], x_vel[i] , y_vel[i], x_accel[i], y_accel[i])
        push!(solution_vectors,solution)
    end
    optimal_solution = objective_value(model)
    return solution_vectors, optimal_solution
end

total_time = 20 
obstacle_scale = 1.2
obstacle_radius = 2
_discretization_steps = 6
_collision_indexes = []
_obstacle = CircularObstacle(4,4, obstacle_radius*obstacle_scale)

_time_intervals = fill(total_time/_discretization_steps, _discretization_steps)
_initial_state = StateVector(0,0,0,0,0,0)
_final_state = StateVector(10,10,0,0,0,0)
speed_max = 3
accel_max = 1

solution, optimal_solution = solveIterativeVehicleMilp(_initial_state, _final_state, _discretization_steps, _obstacle, _time_intervals, _collision_indexes)

x_coords = [point.x for point in solution]
y_coords = [point.y for point in solution]

obstacle_polygon_x_coordinates, obstacle_polygon_y_coordinates = generatePolygon(8, _obstacle.x , _obstacle.y, _obstacle.radius)

# Generate theta values
theta = range(0, stop=2π, length=100)

# Calculate x and y coordinates of the circle
x_circle = _obstacle.x .+ (obstacle_radius*obstacle_scale) * cos.(theta)
y_circle = _obstacle.y .+ (obstacle_radius*obstacle_scale) * sin.(theta)

plot(obstacle_polygon_x_coordinates, obstacle_polygon_y_coordinates, seriestype=:shape, aspect_ratio=:equal, color=:green)
plot!(x_coords, y_coords, seriestype=:scatter, aspect_ratio=:equal, color=:blue)
plot!(x_coords, y_coords, aspect_ratio=:equal, color=:blue)
plot!(x_circle, y_circle, aspect_ratio=:equal, legend=false)
plot!([0,final_x], [0,final_y], seriestype=:scatter, aspect_ratio=:equal, markershape=:x, markersize=10)

println("Checking for collision")
_collision_indexes = checkCollision(solution, _obstacle)
_time_intervals = computeTimeIntervals(_collision_indexes, _time_intervals)
_discretization_steps = length(_time_intervals)

iteration_number = 1
while length(_collision_indexes) != 0
    println("**************************************************************************")
    println("Iteration number: ", iteration_number)

    solution, optimal_solution = solveIterativeVehicleMilp(_initial_state, _final_state, _discretization_steps, _obstacle, _time_intervals, _collision_indexes)
    x_coords = [point.x for point in solution]
    y_coords = [point.y for point in solution]

    # Calculate x and y coordinates of the circle
    x_circle = _obstacle.x .+ (obstacle_radius*obstacle_scale) * cos.(theta)
    y_circle = _obstacle.y .+ (obstacle_radius*obstacle_scale) * sin.(theta)
    
    plot(obstacle_polygon_x_coordinates, obstacle_polygon_y_coordinates, seriestype=:shape, aspect_ratio=:equal, color=:green)
    plot!(x_coords, y_coords, aspect_ratio=:equal, color=:blue)
    p = plot!(x_coords, y_coords, seriestype=:scatter, aspect_ratio=:equal, color=:blue)    
    plot!(x_circle, y_circle, aspect_ratio=:equal, legend=false)
    plot!([0,final_x], [0,final_y], seriestype=:scatter, aspect_ratio=:equal, markershape=:x, markersize=10)
    display(p)

    _collision_indexes = checkCollision(solution, _obstacle)
    _time_intervals = computeTimeIntervals(_collision_indexes,_time_intervals)
    _discretization_steps = length(_time_intervals)
    println("New time interval given by")
    println(_time_intervals)
    println(_collision_indexes)
    iteration_number += 1 
    break;
end 
