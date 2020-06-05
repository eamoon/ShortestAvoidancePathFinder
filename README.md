# ShortestAvoidancePathFinder

A powerful obstacle avoidance shortest path finder. It contains two ways of pathfinding which are direct or along obstacles pathfinding.

In addition, the project includes a visualization tool.

## Attribute

### direct path
The direct shortest path refers to the path formed by connecting the vertices of the obstacle directly.

![img](https://github.com/eamoon/ShortestAvoidancePathFinder/blob/master/Img/edgewaterpath.PNG?raw=true)

Essentially, it builds a visibility graph cache, and if the number of obstacle points is excessive, the build time will be longer, but pathfinding will be faster later. Changes to the obstacle will cause the visibility graph to be rebuilt.

![img](https://github.com/eamoon/ShortestAvoidancePathFinder/blob/master/Img/edgewatergraph.PNG?raw=true)

It also supports pathfinding in obstacle holes.

![img](https://github.com/eamoon/ShortestAvoidancePathFinder/blob/master/Img/holepath.PNG?raw=true)

### along path
The along shortest path is the path to find the way along the edge of the obstacle, and its interval obstacle path is determined by the direction specified.

![img](https://github.com/eamoon/ShortestAvoidancePathFinder/blob/master/Img/alongpath.PNG?raw=true)

Its raw path is as follows.

![img](https://github.com/eamoon/ShortestAvoidancePathFinder/blob/master/Img/directedpath.PNG?raw=true)

The code implementation for this project uses NetTopologySuite and QuickGraph.