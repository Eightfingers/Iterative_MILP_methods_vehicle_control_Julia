using JuMP
using HiGHS
using Plots

function generate_polygon(k, x, y, radius)
    polygon_x_coordinates = zeros(Float64, k)  # Preallocate arrays
    polygon_y_coordinates = zeros(Float64, k)
    for i in 1:k 
        polygon_x_coordinates[i] = x + radius*sin(2*pi*i/k)
        polygon_y_coordinates[i] = y + radius*cos(2*pi*i/k)
    end
    return polygon_x_coordinates, polygon_y_coordinates
end

model = Model(HiGHS.Optimizer)
speed_max = 3
accel_max = 1

k = 10  # Discrete amount
uniform_time_step = fill(0.1, k)
final_x = 10
final_y = 10
M = 1000

poly_d = 8  # Polygon discretization amount
absolute_polygon_x_constraint, absolute_polygon_y_constraint = generate_polygon(poly_d,0,0,1)
rhs_polygon_inequality = zeros(Float64, poly_d)
for i in 1:poly_d 
    rhs_polygon_inequality[i] = cos(pi/poly_d)
end

obstacle_x = 5
obstacle_y = 5
obstacle_radius = 2
obstacle_polygon_x_coordinates, obstacle_polygon_y_coordinates = generate_polygon(poly_d,obstacle_x,obstacle_y,obstacle_radius)

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

# Define objective function
@objective(model, Min, sum(z_ax[i] for i in 1:(k-1)) + sum(z_ay[i] for i in 1:(k-1)))

# Absolute values constraint
@constraints(model, begin z_ax >= x_accel; z_ax >= -x_accel end)
@constraints(model, begin z_ay >= y_accel; z_ay >= -y_accel end)

# Initial Vehicle Conditions 
@constraint(model, x[1] == 0 )
@constraint(model, y[1] == 0 )
@constraint(model, x_vel[1] == 0 )
@constraint(model, y_vel[1] == 0 )
@constraint(model, x_accel[1] == 0 )
@constraint(model, y_accel[1] == 0 )

# Final vehicle Conditions
@constraint(model, x[k] == final_x)
@constraint(model, y[k] == final_y)

# Speed / Accel constraint
for i in 1:k
    for j in 1:poly_d
        @constraint(model, (absolute_polygon_x_constraint[j] * x_vel[i] + absolute_polygon_y_constraint[j] * y_vel[i]) <= speed_max)
        @constraint(model, (absolute_polygon_x_constraint[j] * x_accel[i] + absolute_polygon_y_constraint[j] * y_accel[i]) <= accel_max)
    end
end

# Dynamics constraints
for i in 1:k-1
    @constraint(model, x[i+1] == x[i] + x_vel[i])
    @constraint(model, y[i+1] == y[i] + y_vel[i])
    @constraint(model, x_vel[i+1] == x_vel[i] + x_accel[i])
    @constraint(model, y_vel[i+1] == y_vel[i] + y_accel[i])
end

# Obstacle constraints
for i in 1:k
    for j in 1:poly_d
        @constraint(model, (x[i] .- obstacle_x) .* sin(2*pi*j/poly_d) + (y[i] .- obstacle_y) .* cos(2*pi*j/poly_d) >= obstacle_radius .- M*z_avoid[i,j])
    end
end
@constraint(model, [j=1:k], sum(z_avoid[j, k] for k in 1:8) <= poly_d - 1)

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
    println("********* ", i," **********")
    println("X: ", x_sol[i], " Y: ", y_sol[i])
    println("x_vel: ", x_vel[i], " y_vel: ", y_vel[i])
    println("x_accel: ", x_accel[i], " y_accel: ", y_accel[i])
end

accel_sum = 0
for i in 1:k 
    accel_sum += abs(x_accel[i]) + abs(y_accel[i])
end

println("Total effort ", accel_sum)

plot(obstacle_polygon_x_coordinates, obstacle_polygon_y_coordinates,seriestype=:shape, aspect_ratio=:equal, color=:orange)
plot!(obstacle_polygon_x_coordinates, obstacle_polygon_y_coordinates,seriestype=:scatter, aspect_ratio=:equal)
plot!(x_sol, y_sol, aspect_ratio=:equal)
plot!(x_sol, y_sol,seriestype=:scatter, aspect_ratio=:equal)
plot!([0,final_x], [0,final_y], seriestype=:scatter, aspect_ratio=:equal, markershape=:x, markersize=10)