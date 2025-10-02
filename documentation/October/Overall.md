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

