The below is the output of a CPLEX solver for $7*7$ grid, with some obstacles, but the grid size is the main deciding factor for deciding if the problem is solvable or not (Do we need to prove this?)


| Metric                 | Value                                            |
| ---------------------- | ------------------------------------------------ |
| Binary vars            | 5,163                                            |
| Rows / constraints     | 15,728                                           |
| Nonzeros               | 66,880                                           |
| Presolve eliminated    | 46,368 rows, 13,597 cols                         |
| Root LP solution       | 0.06 sec                                         |
| Branch-and-bound nodes | 1,737                                            |
| Total LP iterations    | 15,994 ticks                                     |
| Solver time            | 17.47 sec                                        |
| Best integer solution  | 40.0                                             |
| Optimality gap         | 0% (integer optimal)                             |
| Coverage               | 40/49 → 81.63%                                   |
| Cuts applied           | Clique:5, Imp. bound:313, Zero-half:30, Gomory:1 |




'''
1. Setting up grid and parameters...
Using optimization model: maximize_coverage
Constraints config: {'eq2': 1, 'eq3': 1, 'eq4': 1, 'eq5': 1, 'eq6': 1, 'eq7': 1, 'eq8': 1, 'eq9': 1, 'eq10': 1, 'eq11': 1, 'eq12': 1, 'eq13': 1, 'eq14': 1, 'eq15': 1}
2. Initializing variable helper and objective function...
3. Computing neighborhood sets (Sensing/Communication)...
4. Building common constraint matrices with toggles...
4a. Building model-specific constraints...
5. Defining variable bounds and types...
Battery constraints are active. Setting battery variable types and bounds.
6. Combining all constraints...
7. Handing off to CPLEX solver...
Starting CPLEX solver (timelimit: 600s, mipgap: 0.01)...
Version identifier: 22.1.1.0 | 2022-11-28 | 9160aff4d
CPXPARAM_Read_DataCheck                          1
CPXPARAM_Threads                                 4
CPXPARAM_TimeLimit                               600
CPXPARAM_MIP_Tolerances_MIPGap                   0.01
Tried aggregator 2 times.
MIP Presolve eliminated 4654 rows and 1560 columns.
MIP Presolve modified 25 coefficients.
Aggregator did 490 substitutions.
Reduced MIP has 54444 rows, 18207 columns, and 195017 nonzeros.
Reduced MIP has 18150 binaries, 0 generals, 0 SOSs, and 0 indicators.
Presolve time = 0.25 sec. (305.81 ticks)
Found incumbent of value 31.000000 after 0.52 sec. (627.75 ticks)
Probing fixed 12845 vars, tightened 0 bounds.
Probing changed sense of 1942 constraints.
Probing time = 5.22 sec. (4440.02 ticks)
Tried aggregator 2 times.
MIP Presolve eliminated 38156 rows and 12850 columns.
MIP Presolve modified 479 coefficients.
Aggregator did 3 substitutions.
Reduced MIP has 16285 rows, 5354 columns, and 69502 nonzeros.
Reduced MIP has 5350 binaries, 0 generals, 0 SOSs, and 0 indicators.
Presolve time = 0.17 sec. (205.97 ticks)
Probing fixed 187 vars, tightened 0 bounds.
Probing time = 0.51 sec. (533.68 ticks)
Tried aggregator 1 time.
Detecting symmetries...
MIP Presolve eliminated 557 rows and 187 columns.
MIP Presolve modified 19 coefficients.
Reduced MIP has 15728 rows, 5167 columns, and 66880 nonzeros.
Reduced MIP has 5163 binaries, 0 generals, 0 SOSs, and 0 indicators.
Presolve time = 0.09 sec. (87.65 ticks)
Probing time = 0.02 sec. (12.11 ticks)
Clique table members: 88346.
MIP emphasis: balance optimality and feasibility.
MIP search method: dynamic search.
Parallel mode: deterministic, using up to 4 threads.
Root relaxation solution time = 0.06 sec. (70.54 ticks)

        Nodes                                         Cuts/
   Node  Left     Objective  IInf  Best Integer    Best Bound    ItCnt     Gap

*     0+    0                           31.0000       49.0000            58.06%
      0     0       49.0000    60       31.0000       49.0000      462   58.06%
*     0+    0                           40.0000       49.0000            22.50%
      0     0       48.3333    54       40.0000   ZeroHalf: 3     1400   20.00%
      0     0       48.0000    44       40.0000      Cuts: 92     1545   20.00%
      0     0       48.0000    59       40.0000      Cuts: 75     1669   20.00%
      0     0       48.0000    39       40.0000   ZeroHalf: 8     1696   20.00%
      0     0       48.0000    49       40.0000      Cuts: 65     1825   20.00%
Detecting symmetries...
Detecting symmetries...
      0     2       48.0000    40       40.0000       48.0000     1825   20.00%
Elapsed time = 9.27 sec. (8273.89 ticks, tree = 0.02 MB, solutions = 2)
      2     3       48.0000    49       40.0000       48.0000     2308   20.00%
      7     3       47.3583    67       40.0000       47.7500     3758   19.37%
     20     5       43.9310    75       40.0000       46.0000     5006   15.00%
     61    24       41.7066    67       40.0000       46.0000     6322   15.00%
    117    24        cutoff             40.0000       46.0000     9371   15.00%
    174     3       45.9091    67       40.0000       46.0000    11716   15.00%
    206     8       44.8667    57       40.0000       45.5000    13528   13.75%
    243     5       44.0000    69       40.0000       44.8667    16896   12.17%
    254    11       43.5000    50       40.0000       44.0000    18216   10.00%
    495   121       41.0000    46       40.0000       43.5000    30778    8.75%
Elapsed time = 12.56 sec. (11695.19 ticks, tree = 0.76 MB, solutions = 2)
    779   254       42.0000    49       40.0000       42.5000    44933    6.25%
   1045   299       41.0000    40       40.0000       42.0000    58297    5.00%
   1375   385       41.0000    41       40.0000       41.8000    76103    4.50%
   1737   160        cutoff             40.0000       41.1190    89975    2.80%

Clique cuts applied:  5
Implied bound cuts applied:  313
Zero-half cuts applied:  30
Gomory fractional cuts applied:  1

Root node processing (before b&c):
  Real time             =    9.22 sec. (8221.37 ticks)
Parallel b&c, 4 threads:
  Real time             =    8.25 sec. (7772.94 ticks)
  Sync time (average)   =    1.76 sec.
  Wait time (average)   =    0.00 sec.
                          ------------
Total (root+branch&cut) =   17.47 sec. (15994.30 ticks)
Solver finished in 17.47 seconds.

--- Final Status ---
Solver Status: integer optimal solution
Objective Value: 40.0

==================================================
OPTIMIZATION RESULTS
==================================================

--- UAV 1 ---
  Path (row, col): [(np.int64(1), np.int64(5)), (np.int64(2), np.int64(5)), (np.int64(4), np.int64(3)), (np.int64(4), np.int64(1)), (np.int64(5), np.int64(2))]
  Battery Levels: [np.float64(100.0), np.float64(98.0), np.float64(96.0), np.float64(94.0), np.float64(92.0)]

--- UAV 2 ---
  Path (row, col): [(np.int64(0), np.int64(3)), (np.int64(0), np.int64(3)), (np.int64(2), np.int64(2)), (np.int64(1), np.int64(1)), (np.int64(3), np.int64(0))]
  Battery Levels: [np.float64(100.0), np.float64(99.9), np.float64(97.9), np.float64(95.9), np.float64(93.9)]

Total Cells Covered: 40 / 49 (81.63%)
t=0 | Coverage: 25.53% (12/47)
/Users/shahidkhan/Documents/btp_work_dir/PathPlanning/mid_term_code/pathplotter_1.py:197: UserWarning: This figure includes Axes that are not compatible with tight_layout, so results might be incorrect.
  plt.tight_layout(rect=[0, 0.1, 1, 1])

Optimization script finished.
(cplex_env) (base) shahidkhan@Shahid-4 mid_term_code % python coverageoptimise.py
1. Setting up grid and parameters...
Using optimization model: maximize_coverage
Constraints config: {'eq2': 1, 'eq3': 1, 'eq4': 1, 'eq5': 1, 'eq6': 1, 'eq7': 1, 'eq8': 1, 'eq9': 1, 'eq10': 1, 'eq11': 1, 'eq12': 1, 'eq13': 1, 'eq14': 1, 'eq15': 1}
2. Initializing variable helper and objective function...
3. Computing neighborhood sets (Sensing/Communication)...
4. Building common constraint matrices with toggles...
zsh: killed     python coverageoptimise.py
(cplex_env) (base) shahidkhan@Shahid-4 mid_term_code % python coverageoptimise.py
5. Setting up grid and parameters...
Using optimization model: maximize_coverage



'''

