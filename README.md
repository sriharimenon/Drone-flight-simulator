# Drone-flight-simulator
Controller, mapping and simulation of a UAV drone through different obstacles.

This repository contains files to simulate the flight of the CrazFlie drone thourgh several simulated obstacles. Mentioned in this Readme are details of the most prominent files and what they do.

# runsim.m
runsim is the master script, and the only script that needs to be run by the user to initiate the simulations. On executing, the command prompt asks the user the course the user wants to simulate flight in from a list of choices.

# controller.m
This file is the cotroller of the drone. A PID cotroller is implemented, which keeps in check the trajectory position, velocity and attitude of the drone. The desired trajectory parmeters are pre-calculated.

# dijkstra.m
This file employs the dijkstra or A-Star algorithm to find the shortest path between the start and stop positions for a given map.
