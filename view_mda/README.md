# Multi Dimensional Adaptation

Multi-dimensional view adaptation based on the multi-dimension adaptation scheme of [Quality of Experience-driven Multi-Dimensional Video Adaptation](https://mediatum.ub.tum.de/?id=1173314).

The node updates the dynamic reconfigure server of the gstreaming node to control the target bitrate as well as the spatial and temporal resolution.

## Run

```shell
roslaunch view_mda view_mda.launch

# Launch Arguments
role_name               # Name of the ego vehicle
camera_view             # Name of the camera view specified in the sensor json file
image_topic             # Name of the input image topic
prio_topic              # Name of the view prioritization topic
frame_stats_topic       # Name of the frame statistics topic (feedback loop of the gstreaming node)
rate_control_topic      # Name of the gstreaming dynamic reconfigure server
```
