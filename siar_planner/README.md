# A*-guided RRT* planner

`siar_planner` includes an opt-in hybrid planner, `AStarGuidedRRTStar`, which
uses the package's existing ROS1 A* and kinodynamic RRT* implementations:

1. A* searches the occupancy grid supplied by `siar_costmap`. It thins the
   `SC_NEGATIVE_OBS` (`-127`) mask to a one-cell-wide centerline and expands
   only its 8-connected neighboring cells; other map labels are rejected.
   The robot-center world point is transformed through the occupancy-grid
   origin pose to a cell, and expanded nodes use cell centers. Start and goal
   centers inside a negative-obstacle region are snapped to the nearest
   centerline cell in that same connected region. A* uses grid geometry
   directly and does not invoke `SiarModel` or motion integration.
2. Up to `astar_waypoint_count` evenly spaced states are retained from that
   route.
3. RRT* samples the goal at the existing `samp_goal_rate`; on other
   iterations, it samples an A* guide state with probability
   `astar_waypoint_bias`, and otherwise samples the configured search space.
4. RRT* expands with the existing SIAR motion model and rewires nearby nodes
   when a lower-cost connection is found. Both `rrtstar` and the guided
   variant add the mean 0-100 cost at the wheel-contact samples to each
   trajectory edge cost, including candidate and rewired edges. New candidate
   states are selected by blending normalized distance to the sample with
   wheel risk.

The hybrid is selected in `planner_test` with:

```bash
roslaunch siar_planner test_synthetic.launch planner_type:=astar_rrtstar
```

Parameters:

| Parameter | Default | Description |
|---|---:|---|
| `astar_max_expansions` | `100000` | Maximum number of grid cells A* expands per request. |
| `astar_waypoint_bias` | `0.2` | Probability of targeting an A* guide state instead of a random state. |
| `astar_waypoint_count` | `10` | Maximum number of evenly spaced A* states used for guidance. |
| `rewire_radius` | `cellsize_m` | Maximum state-space distance for RRT* rewiring. |
| `rrtstar_safety_weight` | `0.5` | Candidate-selection blend: `0` chooses the closest valid state; `1` chooses the state with lowest wheel cost. |

The A* route is required: if A* cannot produce a path, the hybrid returns
`-4.0` and does not start RRT*. Existing `rrt` and `rrtstar` planner types
remain available.

This is an integration with the ROS package's occupancy-grid, motion, and
collision models, not a feature-for-feature port of
`Planificador_SIAR/Planificador_A_RRT_estrella.py`. In particular, it does not
add that script's image-based map segmentation, variable-width state, or
support-polygon stability checks.
