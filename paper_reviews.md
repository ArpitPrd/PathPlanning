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

## Adaptive UAV-Trajectory Optimization Under Quality of Service Constraints: A Model-Free Solution

What is the Core Problem?

Autonomous UAVs need to navigate from a starting point to a destination without human help, especially in unfamiliar and changing environments. This process, called 

path planning, is a complex balancing act. The drone must find an optimal route while considering factors like:

    Safety: Avoiding obstacles (buildings, trees, other drones).

Efficiency: Minimizing path length, energy consumption, and travel time.

Constraints: Adhering to no-fly zones, altitude limits, and payload capacity.

A New Way to Classify Path Planning Methods

A key contribution of this paper is a new taxonomy, or classification system, for organizing the different approaches to UAV path planning. It divides them into four main categories.

    Time-Domain Based: This is about when the path is calculated.

        Offline: The entire path is computed before the mission starts. This is good for known environments but can't adapt to unexpected changes.

Online: The path is generated and adjusted in real-time as the UAV flies, using sensor data. This is essential for dynamic environments with moving obstacles.

Space-Domain Based: This relates to the dimensions of the planned path.

    2D: The path is planned on a flat plane (x, y coordinates), ignoring altitude changes. This is simpler but less realistic.

3D: The path considers horizontal and vertical dimensions (x, y, z coordinates), allowing the UAV to navigate over and around objects.

Environment-Based: This categorizes methods by the type of environment they are designed for.

    Static: The environment is fixed and predictable, with no moving obstacles.

Dynamic: The environment changes over time, with moving objects or unpredictable hazards.

Learning-Based: This focuses on AI-driven techniques.

    Machine Learning (ML): Uses data to identify objects and patterns to help with navigation.

Deep Learning (DL): A subset of ML that uses complex neural networks to learn from vast amounts of data, enabling more advanced decision-making. This category includes Reinforcement Learning.

The Evolution of Path Planning Techniques

The paper traces the historical development of path planning methods, highlighting a shift from rigid algorithms to adaptive, learning-based systems.

Classical and Heuristic Approaches (The Foundation)

    Classical Methods: These are the earliest approaches, often used for static environments.

        Sampling-Based: Methods like RRT (Rapidly-Exploring Random Tree) and PRM (Probabilistic Roadmaps) explore a space by creating a random tree or map of possible paths.

Graph-Based: Algorithms like A* and Dijkstra treat the environment as a grid and find the shortest path between nodes.

Bio-Inspired: Techniques like Ant Colony Optimization (ACO) and Genetic Algorithms (GA) mimic natural processes to find optimal solutions.

Heuristic Approaches: These are rule-based improvements on classical methods, like the A* algorithm, designed to find solutions more quickly in complex scenarios.

Learning-Based Approaches (The Modern Era)

These methods have become popular because they allow a UAV to learn from data and adapt to dynamic situations without being explicitly programmed for every possible scenario.

    Deep Learning (DL): Uses multi-layered neural networks to process complex sensor data (like camera feeds) to understand the environment and make navigation decisions.

Deep Reinforcement Learning (DRL): This is a state-of-the-art hybrid approach and a major focus of the review. A DRL agent (the UAV) learns through trial and error. It receives positive "rewards" for actions that bring it closer to its goal safely and negative rewards for collisions or bad moves. Over many training episodes, it learns a "policy" to navigate successfully on its own.

    Popular DRL algorithms mentioned include 

    DQN (Deep Q-Network), PPO (Proximal Policy Optimization), and Actor-Critic methods.

Key Challenges and Future Directions

Despite major advances, the paper concludes that significant challenges remain before UAVs can navigate fully autonomously in any environment.

    Unknown Threats: Dealing with completely unpredictable hazards in dynamic environments (e.g., a sudden gust of wind, a falling tree branch) is a major hurdle.

Energy and Payload: UAVs have limited battery life and can only carry a small number of sensors and processors, which restricts flight time and onboard computational power.

Real-Time Response: The need to process sensor data and make decisions instantly to avoid collisions is computationally demanding.

Lack of Datasets: Training robust deep learning models requires massive, diverse datasets of UAV flight scenarios, which are currently lacking.

The authors suggest that future research should focus on developing more efficient DRL algorithms, creating lightweight sensors, and designing standardized benchmark solutions to better compare different path-planning techniques.