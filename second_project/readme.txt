RELEVANT INFO:

- Maps folder contains "st_map.pgm" which is obtained doing mapping with slam toolbox. "map_nav.png" is the edited map used for simulation in stage.

- "map_nav.png" is a modified map used in the stage simulation. I cleared noisy pixel around some walls, I cleared most of the noise in the room on the bottom right of the map and I made some walls of the the narrow corridor slightly thinner to help the robot navigate in that area.

- In amcl I put odom_model_type as "omni-corrected". This however required to lower by approx. 2 orders of magnitude the odom_alpha1-5 parameters.

- Dwa wasn't performing very well so I switched to Teb. However I was having lot of troubles with the computational effort and I was getting errors like "control loop missed its desired rate of 5hz" which made the robot stop moving. At first I fixed this problem by reducing the size, resolution, and update frequency of the local map. 

- Then I discovered that setting the parameter "enable_homotopy_class_planning: False" considerably reduced the computational effort, so I was able to revert the changes made to local map size, resolution and update frequency.

- Given that the robot has to navigate some tight spaces while avoiding obstacles some hyperparameter tuning was needed. Some changes I did were: setting "cost_scaling_factor = 30" and "inflation_radius = 0.3" in the costmap_common_params.yaml.  

- I was still having some problems to navigate in the narrow corridor in the middle of the map. After some experiments I was able to make the robot navigate there by setting "inflation_dist = 0.1" and most importantly "penalty_epsilon = 0.02" in the teb_local_planner_params.yaml.
