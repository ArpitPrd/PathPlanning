# PathPlanning
B. Tech project for RL based Path Planning for various drones

## How to Run the Code

```
python3 coverageoptimise.py --config config.json
```

Rest you can toggle from config.json

## Docs for config.json file

- Supports the following toggling mechanism
    - Toggle all the equations (turn them on/ off as per req)
    - toggle the maximisation function 
        - Maximise Coverage with bound of battery consumption 
        - Minimise battery consumption with a min bound on the coverage area

- model name has two values possible as of now: maximize_coverage, minimize_battery

- maximize coverage:
    - worked at battery level B_max = 80
- minimize battery consumption:
    - worked at C_min = 5 (because on an average the consumption was 7)


config.json File Description

This file contains all the necessary parameters to define the UAV mission scenario, configure the solver, and select the optimization model.
Grid and Scenario Parameters

    "sensing_radius": (Integer) The radius in grid cells that a UAV can survey from its current position.

    "comm_radius": (Integer) The communication radius in grid cells required for UAV-to-UAV or UAV-to-base-station links.

    "col_size" & "row_size": (Integer) The number of columns and rows that define the grid's dimensions.

    "N": (Integer) The total number of UAVs available for the mission.

    "T": (Integer) The total number of discrete time steps in the mission horizon.

    "row_sink" & "col_sink": (Integer) The 1-indexed (row, column) coordinates of the base station (sink).

    "coords_obs": (Array of Arrays) A list of 1-indexed [row, column] pairs specifying the locations of obstacles. UAVs cannot fly through these cells.

Battery Parameters

This object contains all parameters related to UAV energy consumption and capacity.

    "b_mov": (Float) The energy consumed when a UAV moves one grid step.

    "b_steady": (Float) The energy consumed when a UAV remains idle (hovers) for one time step.

    "b_full": (Float) The maximum battery capacity of a single UAV.

    "initial_battery": (Float) The battery level of each UAV at the start of the mission (time k=0).

    "ebase": (Float) The minimum required battery level that a UAV must maintain throughout the mission.

Solver Configuration

    "time_limit": (Integer) The maximum time in seconds that the optimization solver is allowed to run.

    "mipgap": (Float) The relative MIP gap tolerance for the solver. The optimization stops when the gap between the best found integer solution and the best possible objective value is less than this fraction (e.g., 0.01 for 1%).

Optimization Model

This object allows you to switch between the two mission objectives.

    "name": (String) Specifies the optimization model.

        "maximize_coverage": Model 1. Aims to maximize the total number of covered grid cells, subject to a total energy budget for the fleet.

        "minimize_energy": Model 2. Aims to find the most energy-efficient paths, subject to a minimum required number of covered cells.

    "B_max": (Float) Used only with "maximize_coverage". Defines the maximum allowable total energy consumption for the entire fleet.

    "C_min": (Integer) Used only with "minimize_energy". Defines the minimum number of grid points that must be covered.

Constraint Toggles

This object contains boolean flags (1 for enabled, 0 for disabled) to selectively activate or deactivate constraints defined in the mathematical model. This is primarily used for debugging or running simplified scenarios. Each key corresponds to an equation number in the provided PDF (e.g., "eq2" toggles the unique position constraint).

## @GodShahid please help me in this

- 1: please help verify the equations if they make sense to you (I have verified the maximise coevrage with battery consumption upper bound, you just have to do the other)
- 2: please help verify each equation with the code, if they are coherent
- 3: please see if the maximisation equations are correctly coded
- 4: Please add your ideas further from here

Till then I will start with the PPT


## Points for the Presentation (Mid-Sem)

- Write the random benchmark (what we plan to exceed and how) 