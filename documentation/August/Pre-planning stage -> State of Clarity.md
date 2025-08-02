Thinking: 
I want to reach a research paper based on this, I hope this can be a bit helpful. So, the drone system, how can I really work on this. Is this even a research paper type topic? I really doubt how is it going to go around, but the first thing is, let us start studying.

Col-UAV -> Collaborative Unmanned Aerial Vehicles
We maintain a common Q-table, for storing the information of their movement

Q-table:
- Each row represents a possible state in the environment
- Each column represents a specific action that an agent takes from state A
- Each cell contains a Q-value, also called as an action-value pair

**Q-Values:** A Q-value, denoted as Q(s,a), represents the estimated "quality" or expected cumulative future reward of taking a specific action (a) in a particular state (s). The higher the Q-value, the better that action is considered in that state for maximizing long-term rewards.

Q learning is a good choice in case of dynamic learning environment, where they need to learn autonomously. >  Each robot in a collaborative group shares the sensed environmental data immediately with its neighbors in [7].
[[RL_path_planning_collaborative_UAVs.pdf#page=2&selection=1,24,3,17|RL_path_planning_collaborative_UAVs, page 2]]

Problem definition:
There are k drones deployed {$D_{1}$, $D_{2}$, $D_{3}$ ..... $D_{k}$}
The area of interest (AoI) is presented as a square divided into m × n grids, where m, n ∈ N . The coordinate set of all grid cells is denoted by cx,y , where cx,y = {c1,1, c1,2..., ci,j , ...}. Here, we assume the size of each grid is the same and equal to the UAV communication coverage. UAVs are flying over the AoI at a fixed altitude h, trying to maximize the number of devices can be covered within the minimum flying time Tmax. UAVs start their mission from the initial starting position, i.e., SPi, where i is the UAV index, i.e., i = 1, 2, .... The position of the ith UAV at time step t is defined as U t i = (xt i, yt i , h). All UAVs stop at the common final position Pf as shown in Fig. 1

At time t, agents observe state ${s_t}$ ∈ S, perform an action at ∈ A, and consequently receive some reward ${r_t}$ ∈ R . The increment in time propagates the agent to the new state $s_{t+1}$ in an environment where this cycle restarts.

Description of the Q-learning framework:
We assume that all the UAVs are identical, having the same set of states and actions with them. 
Q(si, ai) = Q(si, ai)+α∗(Rt i +γ ∗Q(s′ i, a′ i)−Q(si, ai))

**Our Metrics are going to be:**
1. minimize the flying time
2. Maximize the area coverage by the sensors
3. Path smoothness

Minimizing the flying time also minimizes the fuel consumption. 
It also includes a factor of reducing as much redundancy as possible

Can we know how many UAVs can be synchronized together, but its just a research, until we get a research out, it may have been upgraded a lot....

**Do you want us to?**
1. Work on the quality of a particular area covered?
2. Can we try some red zoning areas, some blue zoning areas, etc. because my manipulating the reward function, we can prolly try that?
3. I don't want to get into electronics of it? Like dependencies on the battery life ending and all.
4. Let us define some no fly zones, where the drones should not fly, or maybe that is an obstacle... Or should they be kept separate, that also needs to be thought out




There are again two main methods -> A* and the dynamic algorithm
Let us try to study the A* algorithm, and trying to understand it with the brain I have got [[A*_understanding]]
Also, I think I should understand the current dynamics, of what kinds of researches are being held across the world [[dynamics]]





