Note: There are two ways to implement any of the 4 algorithms along with the 3 given maps. This can be done by using either of codes 'search_code_1.py' or 'search_code_2.py'
If a new map is to be used, than use 'search_code_2'

The required codes and maps are as follows:
search_code_1.py
search_code_2.py
map0.txt
map1.txt
map2.txt


>>>>>>
Method - 1 : using search_code_1.py : to use the specified combinations

Here, I have typed the code which includes all the necessary functions to use the algorithms. When running, the code will ask for a number for a 'combination of algorithm and a map', and also for the set of actions required.
I have used 15 numbers for 15 combination of:
3 maps:
map0, map1, map2
5 algorithm cases:
depth_first_search
breadth_first_search
uniform_cost_search
a_star_search with manhattan heuristic
a_star_search with euclidean heuristic

And, there are two action sets
Set 1: u, d, l, r
Set 2: u, d, l, r, ne, nw, se, sw

To Use the code:
1. Open/Run search_code_1.py
2. The code ask for a number to run a combination of an algorithm and a map.
3. Enter the required value
4. The initial map will be displayed
5. After closing the map-display window, the code will display a new map showing the visited nodes and also the path taken in appropraite colors.
6. Also, the path, visited states and action sets will be displayed in the print display window.


>>>>>>
Method - 2 : using search_code_2.py : to use user required combinations or a new map

Here, I have typed functions, classes and data structures first. I have typed code lines to read a map and to call the required algorithm function at the end of the same code,
The default code line for reading a map is given as:
g= GridMap('./map0.txt')
To use any map of your choice, replace './map0.txt' with any other map in the .txt file format

The default code line for reading a map is given as
[[path,action_path],visited] = dfs(g.init_pos, g.transition, g.is_goal, _ACTIONS, g.goal)
To use any other algorithm of your choice, replace 'dfs' in the above line with
'dfs' :for Depth First Search algorithm
'bfs' :for Breadth First Search algorithm
'uniform_cost_search' :for Uniform Cost Search algorithm
'a_star_search_euclidean' :for A Star Search algorithm with euclidean distance heuristic
'a_star_search_manhattan' :for A Star Search algorithm with euclidean distance heuristic

To Use the code:
1. Open search_code_2.py
1. Use the map and algorithm function of your choice as mentioned in the above paragraph
eg: for using map1 with Uniform Cost Search, your codelined should be:
g= GridMap('./map1.txt')
[[path,action_path],visited] = uniform_cost_search(g.init_pos, g.transition, g.is_goal, _ACTIONS, g.goal)
eg:
to implement the 'depth first search' algorithm with 'map1.txt', do the following
1. Open search_code.py
2. To use 'map1.txt' edit the 2nd line of the code g= graph_search.GridMap('./map_number.txt')). Here, replace './map_number.txt' with 'map1.txt'
3. After making the required changes, Run the code
4. The initial map will be displayed
5. After closing the map-display window, the code will display a new map showing the visited nodes and also the path taken in appropraite colors.
6. Also, the path, visited states and action sets will be displayed in the print display window.
