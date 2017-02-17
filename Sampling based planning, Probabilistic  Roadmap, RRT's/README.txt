I have created two seperate code files each for RRT and PRM. Although PRM.py consists of RRT, PRM and the A*star classes and was meant to combine both RRT and PRM solutions, however the A*search algorithm gave a problem and hence a seperate RRT and PRM file.

In the RRT.py file,
change a map, change env1.txt to any other map at line 236
The boolean value of connect on line 236 controls either simple RRT or Bidirectional RRT
IF True: the bidirectional RRT will run
If False: the Simple RRT will run

to use RRT connect method
we have to change the boolean value of connect at line 99, to use simple RRT as RRT connect
or change boolean value of connect at line 118 to use RRT connect for bi directional search


In the PRM.py file
on line 745, step length copntrols the distance mooved for testing in a local planner
on line 745, dist_to_nearest_neighbour controls the distance to nearest neighbour from a sample point
line 517 holds the build_prm function
at the end of the unction I have commented plan which takes the value from a* search function. I have tried to link the sampled points along with their edges to the a* search algorithm, however I got errors and hence I returned the plan as None.
However, the sampled nodes, linking with the nearest nodes is displayed when the code is run