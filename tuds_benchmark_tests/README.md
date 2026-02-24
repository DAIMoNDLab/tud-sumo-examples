# TUD-SUMO Benchmark

This repository contains the benchmark tests for [TUD-SUMO](https://github.com/DAIMoNDLab/tud-sumo), a research-oriented wrapper for the microscopic traffic simulation package SUMO. The tests compare TUD-SUMO to only using standard TraCI for accomplishing the same tasks.

## Tests

The tests all use a simple on-ramp network scenario, lasting 2000 steps, and aim to cover most base uses of TUD-SUMO. A statistics file is generated in each test along with any other test-specific output. They are as follows:

  1. Simple run of the simulation without any interaction.
  2. Generation of floating-car data from a simple run of the simulation without any interaction.
  3. Generation of a space-time diagram from the minimum required amount of simulation data.
  4. A run of the on-ramp scenario with ramp metering applied (ALINEA), demonstrating traffic signal management and interaction with the simulation.
  5. A run of the on-ramp scenario with an incident active on the downstream segment.

## Test Metrics

|       Metric      |                                             Definition                                            |   Unit  |
|:-----------------:|:-------------------------------------------------------------------------------------------------:|:-------:|
|    `runtime_s`    | Wall-clock elapsed time, including all python overhead, waiting, input/output etc.                | Seconds |
|   `step_runtime`  | Wall-clock elapsed time per simulation step.                                                      | Seconds |
|    `cpu_user_s`   | CPU time spent executing user-level code/libraries.                                               | Seconds |
|   `cpu_system_s`  | CPU time spent on system/kernel operations, such as input/output, memory management etc.          | Seconds |
|   `cpu_total_s`   | Sum of user & system CPU time, resulting in the total processing time consumed by the process.    | Seconds |
| `cpu_avg_percent` | Average CPU utilisation of process over wall-clock runtime.                                       |    %    |
|   `peak_rss_mb`   | Peak memory usage, or maximum recorded resident set size during execution.                        |    MB   |
|   `output_size`   | Size of all files generated during the test.                                                      |  Bytes  |
|       `loc`       | Number of lines of code in the function.                                                          |    -    |
|    `api_calls`    | Total number of function calls to TraCI or Simulation/Plotter objects (including repeated calls). |    -    |
|    `complexity`   | Code cyclomatic complexity, counting decision points/paths through source code.                   |    -    |


## Running Tests

After cloning the repository, run the tests using the commands below (either order). This will save the test results into `results/base.csv` and `results/tuds.csv`, as well as generate barcharts for comparison.

```
python3 base.py
python3 tuds.py
```