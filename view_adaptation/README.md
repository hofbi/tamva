# View Adaptation

Main module for running traffic aware view adaptation.

This requires the a running Carla server unless the `bag_input` option is set to `true`.

## Run

### Multi Server

* Run a six camera driving setup with gstreaming server.
* Connect to the server via the `telecarla_multi_client.launch` launch file.

```shell
roslaunch view_adaptation view_adaptation_multi_server.launch
```

### Multi Local

* Run a six camera driving setup locally.
* Used for testing the ROI filter.

```shell
roslaunch view_adaptation view_adaptation_local.launch
```

### Single Server

* Run a single camera driving setup with gstreaming server.
* Connect to the server via the telecarla_single_client.launch launch file.
* Used for testing the multi-dimension adaptation

```shell
roslaunch view_adaptation view_adaptation_single_server.launch
```

## Scenario Evaluation

In order to use the view adaptation on top of telecarla while running the different scenarios for evaluation purposes, follow these steps:

### Local Usage
The steps 1. and 2. apply from [here](https://gitlab.lrz.de/teleop/teleop-carla/-/blob/master/telecarla_scenario_runner/README.md), only step 3. changes to:

3.
``` roslaunch view_adaptation view_adaptation_local_scenario_runner.launch town:=$(rosparam get /town) role_name:="hero" ```

### Remote Usage
To use this ROS node on a remote server, these steps should be followed:
1. Ssh to a remote server.
2. Run the simulator:
```CarlaUE4 -RenderOffScreen```
3. From the same remote server, run the docker container:
```docker-compose run view_adaptation```
4. Launch the evaluation node while specifying the arguments if necessary:
``` roslaunch telecarla_scenario_runner telecarla_scenario_evaluation_runner.launch timeout:=30 ```
5. From another terminal on the same remote server, connect to the docker container on which the node from step 4. is running in order to run the view adaptation server node:
```
docker ps
docker exec -it <name-of-view-adaptation-container> /bin/bash

# Single View
roslaunch view_adaptation view_adaptation_single_server_scenario_runner.launch town:=$(rosparam get /town) role_name:="hero"

# Multiple Views
roslaunch view_adaptation view_adaptation_multi_server_scenario_runner.launch town:=$(rosparam get /town) role_name:="hero"
```
6. On the local machine, run the client:
```
# Single View
roslaunch telecarla telecarla_single_client.launch host:=<IP-address-of-the-remote-server> role_name:="hero"

# Multiple Views
roslaunch telecarla telecarla_multi_client.launch host:=<IP-address-of-the-remote-server> role_name:="hero"

```

### Further Remarks
* When working with a remote server, make sure that **gstreaming** is copied inside teleop-carla before running the command ```docker-compose build```.
* It's also possible to use a different port for the RTSP server used by gstreaming in case the default port is held up by another process for the single view usage only:
```
# Server Side View Adaptation
roslaunch view_adaptation view_adaptation_single_server_scenario_runner.launch town:=$(rosparam get /town) role_name:="hero" port:=<Chosen Port>

# Client on Local Machine
roslaunch telecarla telecarla_single_client.launch host:=<IP-address-of-the-remote-server> role_name:="hero" port:=<Chosen Port>
```
* For further comments about the telecarla client as well as the scenario runner used by the view adaptation launch files, please refer [here](https://gitlab.lrz.de/teleop/teleop-carla/-/blob/master/telecarla_scenario_runner/README.md).
