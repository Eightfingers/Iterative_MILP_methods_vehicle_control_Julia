# Iterative MILP methods for vehicle control - Julia 
Done as part of my Linear Optimization class presentation. Contains Julia implementation for the following paper - M. G. Earl and R. D'Andrea, "Iterative MILP methods for vehicle-control problems,". The dynamics of the robot is modelled as a discrete point-mass where at each time step:
 
$x_{pos+1} = x_{pos} + x_{vel}$

$x_{vel+1} = x_{vel} + x_{accel}$

$x_{accel}$ is denoted as $u$ inside the code to symbolize control variables just like in the paper.

# Julia scripts
- 1_simple.jl: Simple Trajectory Generation, discrete point mass test
- 2_obstacle_avoidance.jl: Obstacle (modelled as circles) avoidance
- 3_minimum_time.jl: Minimum time obstacle avoidance trajectory 
- 4_iteartive_method.jl: Iterative MILP as described by the paper to reduce solving time. Collision detection algorithm implementation is different from the paper.
- 5_growing_iterative_method.jl: Increasing of obstacle radius at every iteration (problem relaxation) for even faster solving time.

# Other papers
This repo also contains other papers that cover using Mixed Integer Linear Programming (MILP) for control/trajectory of systems.

# Slide link
Google slides that I used for my Linear Optimization class presentation.
https://docs.google.com/presentation/d/1FoUmzOyAJMmsEuQvcE0ivGW8gXnK6MOEU1Zfq-TU4Rk/edit?usp=sharing