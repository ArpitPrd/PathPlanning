The challenges we are facing right now:
1. The limitation due to the use of the CPLEX
2. How do we model the charging stations
3. Are there any optimisations, or algorithmic things possible when designing the model?
4. The better obstacle modelling, making it togglable for different kind of obstacles, like no fly zones, or including the line of sight?
5. Can we put a bound on the periodic coverage of a mobile location?

Pt 1: Discussing over the challenges we face with the cplex:
1. We will need CPU only HPC. The only thing we can back on:
- The number of threads, to handle branch and bound explosion
- The RAM, so that the memory required never exceeds the main memory we have. (In our case, the memory used was sufficiently low)
- Guess what, the above point is wrong. we need way too more memory to run our constrains, our inequalities constraints array has a size of 2.3 GB for A_ineq ($7*7$ case), and 6.39 GB for a $8*8$.
- Conclusion: We sure as hell need an HPC for it. Even if not for increasing memory, we need to increase the RAM.
The choices we have:
- Ice Lake CPU nodes: 256GB RAM 
- Skylake CPU nodes: 96GB RAM
- Haswell CPU nodes: 62GB RAM 


@GodArpit: Hame GPU ki koi need nahi hai. Check if Badal can give us more RAM, I am not sure how much can increasing the number of threads help? According to GPT, solve time sublinearly decrease karta with number of threads.

[[CPLEX_OUTPUT_STUDY_7x7]]


*ToDo@Shahid: See the implications of approaches like sparse matrices in reducing the memory and space complexity*
https://or.stackexchange.com/questions/13239/solving-a-large-scale-milp-problem-with-cplex-takes-long-time?
This talks about some big M constraints-> can we optimise it? Does sound like a pretty laborious task to me.
https://public.dhe.ibm.com/software/products/Decision_Optimization/docs/pdf/paramcplex.pdf


*cplex would require either admin permission, or else we'll need to ask them to get cplex installed for us*




pt2. Half of the modelling is done for the stations, charging and discharging equations, now, there has to be different stations, so the boolean variable has to be one for only one drone at a time, or let us say each station has a capacity for k drones. 

1. Should we relax the collision constraints -> Depends on if we want to ??
2. Should we go for modelling obstacles more extensively? 
- Blocking LOS?
- Only centre or peripheries

3. Model for different stations has been made


The another current issue is, why is it giving infeasable solutions. Like literally why... It does not make any sense




So, the tasks have been reduced to improving te battery thing, but I am yet unable to understand what the issue is? What is it that it starts giving me linear conflicts after a certain limit, like that too for coverage.
There are some conflicting lower and upper bounds, which I need to look over, But how come they are arising once we have put the equations into higher coverage




Okay, let us try to analyze the case of a $3*3$ matrix. 
Trivial upper bound = Sum(c_i), the largest possible value it can take
What the fuck, C_min is the number of cells. 
A loophole - The sink node is actually not being taken care of being a part of coverage region as of the plotter code


Tasks at hand:
1. Update the equations to sense region -> done
2. Update the equations to show obstacles -> done
3. Maybe update the equations to give one movement at a time






Later tasks:
1. Introduce the dummy drone architecture......
2. Split the recharging stations

