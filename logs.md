## Unlogged Inforamtion

- Decided upon the four pillars: Application, Modelling, Algorithm, Theory
- Have to work  particularly on the Application and Modelling, once that is set, the other 2 are routine
- The paper based on collabrative search does not have a repo, must either be built from scratch or reconstructed with existing repositries

## Day 1

- Plan for today
    - See if tesi can be implemented
    - make ppt
    - read some papers, that use RL [Autonomous UAV Navigation Using Reinforcement Learning](https://arxiv.org/pdf/1801.05086)
- What was learnt
    - RL algs for high level tasks
    - We can represent the environment as a set of traingles, it is a heuristic and has to be treated that way
    
- Blockers:
    - The TESI code, though useful, is showing lots of errors and cannot be corrects
    - one error to resolve is how to convert a .pb file that stores the information about the weight matrices that were trained
- Found a repo that utilizes A* algorithm, very well, have to run it
- Find some more papers
- Make a ppt for presentation to the professor
- Most of the papers are from one of the following applications - RL, A*, RRT*, etc., their application purpose is different.
- Finsihed ppt and reading the paper planned, nothing new found except usage of PID

## Day 2

- produce possible usecases for the UAVs
- Need to figure out a way to model batteries as a new index for battery life associated with each of the models. Ideas that may be used (taken from the discussion today):
    - curr_battery_level = fn(prev_battery_level, time)
    - location co-ordinates for the drones
    - piece wise defined function for fn
    - fit this in MILP

## Day 3

- we have the paper, look at the paper and make some analysis of how things can be done
- write a code to implement the energy model of drones
- understand the code on git

- setup the matlab for linux machines

## Day 4

- think further on the energy idea, try to make a constraint out of it