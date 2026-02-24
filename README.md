# TUD-SUMO Examples & Benchmark Tests

There are 2 different example scenarios included for testing TUD-SUMO:

1. **_Basic Scenario_**: Acts as an integrity check to ensure both SUMO and TUD-SUMO are installed and run correctly.
2. **_A20 Scenario_**: A complex scenario of the A20 in Rotterdam, with multiple controllers and events. To show the gui, run with the command line flag `-gui` and use `-seed {x}` to run with a specific seed.

The scenarios can be run with either `basic_scenario/demo.py` or `a20_scenario/demo.py`. The networks used in each scenario are shown below.

The benchmark tests, copmaring TUD-SUMO and standard TraCI, can be found in the `tuds_benchmark_tests` file, along with a detailed description of the tests and results. 

The full documentation can be found here: [tud-sumo.github.io/docs/](https://tud-sumo.github.io/docs/)

## Basic Scenario

The network is a simple ~300m stretch of road with a constant demand of 1800veh/hr throughout the simulation. If the SUMO GUI (Graphical User Interface) is shown and the simulation is able to run for 1000s, everything should be installed correctly. _Congratulations!_

![Basic scenario network](basic_example/basic_scenario/network.png)

## A20 Scenario

Here, the northern section of the motorway ringroad around Rotterdam, the A20 between Schiedam and the Kralingse Bos, is simulated. This scenario aims to demonstrate the full functionality of TUD-SUMO, with multiple ramp meters, variable speed limit and route guidance controllers, as well as several different events and interactions.

![Example Scenario](a20_example/a20_scenario/network.png)
