### Sampling-Based Planning 

#### The Probabilistic Roadmap (PRM)

The [probabilistic roadmap](https://en.wikipedia.org/wiki/Probabilistic_roadmap) planner is a motion planning algorithm in robotics, which solves the problem of determining a path between a starting configuration of the robot and a goal configuration while avoiding collisions.

- Sampling: We are sampling from a uniform random distribution over the square [-0.5, 0.5] x [-0.5, 0.5]. Since we will always use the same start and goal configurations, we should also choose samples at these configurations, so they are built into the tree. It is up to us to choose the number of nodes in our graph, but we should choose enough that the start and goal are likely to be in the same connected component of the graph.

- Creating edges: It is up to us to choose the number of neighbors $k$ to try to connect to each node.

- Searching the graph: We used A* search, to find the shortest distance from start configuration to the goal configuration.