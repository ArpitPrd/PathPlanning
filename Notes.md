## Applications

1. Street lamp lighting with various charging pods offereing discrete event chargings and replacement street drones
2. Videography for speacial purpose tasks (coal mining, tunnel detours), farming
3. Rescue operations in situations of floods, mines, earthquakes (inside broken buildings)

## Modelling

Two things are to be modelled: 
1. Agent(s):
    - Number of drones
    - max speed, max angle of deflections, max image resoltions, max height of the drone
2. Environment:
    - Area of interests
    - Units of Area

## Algorithm
1. Reuse existing algorithms and apply multi-objective programming to find the number of drones and time taken
2. Improve RL

## Theory (Limitations of this Model towards the Applications)

To find the areas that cannot be visited
## Target KPIs

1. Perform 30% better than random model (simulated earlier following brownian motion) (Notice patterns from here)
2. Improvise speed of algorothm

## What is our final goal (Possibilities)

1. Flying drones for power efficiency, time, duration
2. Flying drones to cover various depths of details at the cost of reduced span frame
3. Exploration of an unknown environment with no prior history

## Ideas that can be used to implement

1. Use LLMs for tracking between machines
2. Multiple agents surving together
3. PYMOO for solving multiobjective problem (time, drones)
4. height and resolution
5. policy and a subpolicy
6. Share between exploration and exploitation ([link](https://dl.acm.org/doi/pdf/10.1145/3477314.3507052))
7. Leader follower coalition to perform certain tasks by certain group of solutions. this can be used as a follow up to our intial approach (by API calls to the lower levels)
8. can work to for disconnectivity of the drones in the middle, store information for a while and then join back.   

## Papers that can be used to refer

| Sl. No. | Paper Link | GitHub Link | Short Description |
| --- | --- | --- | --- |
1 | [A Reinforcement Learning-based Path Planning for Collaborative UAVs](https://dl.acm.org/doi/pdf/10.1145/3477314.3507052) | Not Available | Uses many mobile devices to track info supplied by few drones
2 (very useful) | [Reinforcement Learning approach for cooperative UAVs exploration of critical environments](https://webthesis.biblio.polito.it/secure/19281/1/tesi.pdf) | [github](https://github.com/gbattocletti-riccardoUrb/rl-exploration-for-uavs) | Based on charges and RL 
3 | [Adaptive Informative Path Planning Using Deep Reinforcement Learning for UAV-based Active Sensing](https://arxiv.org/pdf/2109.13570) | [github](https://github.com/dmar-bonn/ipp-rl?tab=readme-ov-file) | combines tree search with an offline-learned neural network predicting informative sensing actions
4 | Path Planning for drones using A* algorithm | [github](https://github.com/abhiojha8/3D-Motion-Planning/tree/master?tab=readme-ov-file) | uses A* algorithm in 3D notion from point A to point B, something called udacity, can look
5 | UAV Motion Planning which includes: A*, RRT, RRT*+Min Snap | [github](https://github.com/peiyu-cui/uav_motion_planning) | Has all possible methods implemented from papers [Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight](https://github.com/peiyu-cui/uav_motion_planning?tab=readme-ov-file), [Search-based Motion Planning for Aggressive Flight in SE(3)](https://arxiv.org/pdf/1710.02748), kdtree-acceleration, [Minimum Snap Trajectory Generation and Control for Quadrotors](https://web.archive.org/web/20120713162030id_/http://www.seas.upenn.edu/~dmel/mellingerICRA11.pdf)
6 | Reinforcement Learning for Autonomous navigation of UAVs, papers: [link](https://arxiv.org/abs/1801.05086), [link](https://storage.googleapis.com/deepmind-media/dqn/DQNNaturePaper.pdf) | Not Available | RL for indoor environments, paper 1 has PID controller
7 | [Multi-objective path planning for multi-UAV connectivity and area coverage](https://www.sciencedirect.com/science/article/pii/S1570870524001318?via%3Dihub) | Not Available | uses NSGA-II to optimise both on coverage and time
8 | [Throughput Maximization in Multi-UAV](https://ieeexplore.ieee.org/document/8476567) | NA | used for maximising the throughput of communication, seems interesting, does power + througput 


## Drone type vs battery life vs costs

| Drone Type | Typical Payload Capacity | Typical Flight Time (Loaded) | Approximate Cost Range |
|---|---|---|---|
| **Consumer Drones** | Up to 2 kg (4.4 lbs) | 25 - 50 minutes | $50 - $2,000 |
| **Commercial/Industrial Drones** | 2 - 10 kg (4.4 - 22 lbs) | 20 - 45 minutes | $5,000 - $20,000 |
| **Heavy-Lift Drones** | 10 - 30 kg (22 - 66 lbs) | 10 - 30 minutes | Over $20,000 |
| **Ultra-Heavy-Lift Drones** | 30 - 200 kg (66 - 440 lbs) | 5 - 20 minutes | Starting at $60,000 |

Citations:
- [Market](https://www.grandviewresearch.com/industry-analysis/drone-market-report)
- [Heavy Lift drones](https://www.grepow.com/blog/what-is-a-heavy-lift-drone.html#:~:text=Unlike%20consumer%20drones%20built%20for,and%20performance%20under%20heavy%20load.)
- [Floght time vs payload](https://www.researchgate.net/figure/Drone-payload-and-flight-time_fig4_348496100)

- fixed wing uavs: only chest span wings, these have a limitation of not being able to move around in a mobile fashion, higher payload
- rotatry wing uavs: have that rotation on the tips of the wings these can move fluently arounf, lower payload, can refer to https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8476567

## Application

- Notice this can almost be anything, once we project it into a model then algorithms may be different
- Most of the applicatoins Modelling boild down to RL/ some sort of optimisation
- contributing in a new application 
- We can improve only one segment of the above stages, mostly possible with 'algorithms'
- We will also keep coverage area + coverage time as opt functions



## Some imp points for later:

- : (i) traveling/movement and sensing ranges of UAVs are limited
due to which multiple UAVs may visit a grid point across different
time-steps to reach uncovered grid points, and (ii) UAVs need to be
within the communication range of at least one other UAV in order
to maintain network connectivity

## Modelling Energy Constraints:

- so we want to make sure it fits the milp class problem formulations
- your grid point becomes (x, y, t), where t is the angle that was used to reach this state
- fn = {
    battery - eps_static; if static
    battery - eps_movement; if t = 0
    battery - eps_movement - eps_turm; if t > 0 
}

- currently modelling the case when the charging pod is the base and instantaneous charging

- for the turn you can start with 45 degree movements, which indicate diagonal movements, this will inroduce some variability to the movements

- the location of the drone is still denoted by (x, y)

- change the maximising function to -> coverage + sigma(b)
    - these two are anti of eachother
    - each drone has another variable the is 
        b(k, n) = {
            eps if battery level changed from prev time stamp
            0 otherwise
        }

- constraint can be:
    - sum across all nodes battery lives > eps_min_to_return_to_base * N
    - need some constraint on replacement policy
    - if sum of battery levels are bounded, this would force some of them, to be static and only a few to be moving

- replacement policy:
    - the moment the battery of a drone drops below eps_min_to_return_to_base
    - it takes the shortest route in an altitude below the surviellance atlitude 
    - we no more consider this drone in the system
    - 
z(k, i, n) denotes the kth time instant, ith grid point, nth drone

## Benchmarking

- Can use existing papers and find out the novel metric that we invented for their scenario and check the performance
- we want to be sure to use something, that is of proper importance.

## Battery Constraints so far:

- some sources to refer: 
    - https://www.sciencedirect.com/science/article/pii/S157087052100072X?via%3Dihub
    - https://www.sciencedirect.com/science/article/pii/S1570870525002148
    - https://ieeexplore.ieee.org/document/8476567
    - https://www.jouav.com/ useful for understanding drones

- Max Battery Limit: b(n, k) <= b_full
- Min Battery Limit: b(n, k) >= e_base
- Turn vs Mov: y_turn(n, k) + y_mov(n, k) <= 1
- Exhange vs NoExchange: Y_exchange(n, k) + Y_noexchange(n, k) <= 1
- Binary Turn Limit: y_turn(n, k) >= 0
- Binary Mov Limit: y_mov(n, k) >= 0
- Binary Exchange Limit: Y_exhange(n, k) >= 0
- Binary NoExchange Limit: Y_noexchange(n, k) >= 0
- Binary Exchange Limit: Y_exchange(n, k) <= z(base_station, n, k)
- battery updatation: b(n, k+1) = Y_noexchange(n, k) * {b(n, k) - y_turn(n, k) * b_turn - y_mov(n, k) * b_mov} + Y_exchange(n, k) * b_full

- objective function: (multi objective): (sigma ci over all i = 1, M), (sigma y_turn(n, k) * b_turn + y_mov(n, k) * b_mov + Y_exchange(n, k) * (b_full - b(n, k)))

## General Pointers:

- you need to git pull to check outhje resourcesgit (note this is to be done in the other branch, not in your)
- meshgrid representation is something native to matlab, (x[i, j], y[i, j]) -> forms meshgrid provide the grid point, can use meshgid for more variety of tasks is is more like a cross product
- unravel from linear to row, col
- 

## MILP Solvers:

- link to some of the solvers: https://stackoverflow.com/questions/26305704/python-mixed-integer-linear-programming
- scipy does milp: https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.milp.html
- cvxpy docs: https://www.cvxpy.org/ (seems useful since targetted specifically towards this)
- MILP with GA: https://github.com/pdimh/mi-lxpm