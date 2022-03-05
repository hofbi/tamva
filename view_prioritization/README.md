# View Prioritization

Prioritize individual camera views based on their position and the vehicle's odometry. The priority is distributed as ratio in percent and as the bitbudget defined by the available network transmission rate.

## Run

```shell
roslaunch view_prioritization view_prioritization.launch

# Launch Arguments
role_name                # Name of the ego vehicle
sensor_definition_file   # Path to the sensor definition file containing the camera positions
```
