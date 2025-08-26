## Work on Paper CCPP

- the goal of the paper is to be able to model relay of information 
    - first formulation: maintain area coverage to the base station while maintaining connectivity
    - second formulation: minimize the number of movements required to achieve the given coverage, again, while maintaining the connectivity 

- A glue that binds you, another that repels you to seek more area
- mobility strategy required
- energy constraints, time constraints, communicagtion constraintes
- introducing three way angles that split the space

- paper on drone battery modelling: [Energy efficient solar power model](http://dx.doi.org/10.1016/j.adhoc.2021.102517)
- paper on wihtout exceeding max flight time: [Total Coverage with connectivity constraints ](http://dx.doi.org/10.1504/IJSNET.2020.109714), can help in modelling the battery
- can introduce buffering of data

- constraints:
    - Position constraint: self existence 
    - collision avoidance: no two drones in same grid 
    - sink connectivity 
    - nodes connectivity (p imp)
    - movement constraints

## From TESI Paper Github implementation

1. Generates end to end images
2. has all models saved
3. need to work on file path names
4. only coverage as of now
5. retrain instead of training all over again
6. Use pymoo opts on top of this to get hold of time and number of drones, assuming this algorithm is gold
7. get the main running tomorrow

## Throughput Maximization in Multi-UAV Enabled Communication Systems With Difference Consideration

- wireless communication through multiple UAVs (our case is surveillance purpose, but the point with multiple uavs is that they can help yield the communication integrity that we want to have.)
- these guys do not assume same UAV, interesting
- maximising the minimum GT Thrugput, given the power constraints, communication scheduling and UAVs trajectory
- optimizatino of solution done through:
    - block coordinate descent, an iterative algorithm
    - relaxation 
    - succesive convex opt techniquea
- can be used in wireless communication and effective netowking
- this paper does something different, it uses uavs more as information collector than information perciever, and deleiver
- no power model, waste
- though, solves mixed integer non convex optimisations

## Joint Trajectory and Power Optimization for UAV Relay Networks 

- link: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8068199
- this does point to point opts
- builds equation for each time slot to optimize the power wrt location of base to uav to mobile
- they have constrained equations for using Pmd + Puav <= Plimit. This is just one of their constraints 

## Physical Internet Hub-Integrated Drone Logistics Model: Optimizing Cost and Energy Efficiency for Urban Delivery Systems

- uses GA to handle the intractability of MILP
- Last mile connectivity 
- GAs often struggles with initial search space exploration (cite this)
- Energy = weight depended + wasted energy when it goes for refilling 
- they use ORS, which have battery replacement mechanism 
- Physical Internet Structure (what?)
- do not have time slotting, time slotting here is point to point delievery
- can use binary decision varibles to switch of constraints, and have constraints on these binary variables 
- we can capture the not producing useful product by minimizing the fact that battery has been wasted in going back to the recharing station
- they perform only one time solve of the entire problem, which drone picks which delievey and goes where