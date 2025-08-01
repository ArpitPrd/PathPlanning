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

## Papers that can be used to refer

| Sl. No. | Paper Link | GitHub Link | Short Description |
| --- | --- | --- | --- |
1 | [A Reinforcement Learning-based Path Planning for Collaborative UAVs](https://dl.acm.org/doi/pdf/10.1145/3477314.3507052) | Not Available | Uses many mobile devices to track info supplied by few drones
2 (very useful) | [Reinforcement Learning approach for cooperative UAVs exploration of critical environments](https://webthesis.biblio.polito.it/secure/19281/1/tesi.pdf) | [github](https://github.com/gbattocletti-riccardoUrb/rl-exploration-for-uavs) | Based on charges and RL 
3 | [Adaptive Informative Path Planning Using Deep Reinforcement Learning for UAV-based Active Sensing](https://arxiv.org/pdf/2109.13570) | [github](https://github.com/dmar-bonn/ipp-rl?tab=readme-ov-file) | combines tree search with an offline-learned neural network predicting informative sensing actions


## From TESI Paper Github implementation

1. Generates end to end images
2. has all models saved
3. need to work on file path names
4. only coverage as of now
5. retrain instead of training all over again
6. Use pymoo opts on top of this to get hold of time and number of drones, assuming this algorithm is gold
7. get the main running tomorrow