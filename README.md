# Traffic-Aware Multi-View Video Stream Adaptation for Teleoperated Driving

[![Actions Status](https://github.com/hofbi/tamva/workflows/CI/badge.svg)](https://github.com/hofbi/tamva)
[![Actions Status](https://github.com/hofbi/tamva/workflows/CodeQL/badge.svg)](https://github.com/hofbi/tamva)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

## Paper

If you use this work please cite our paper.

*Traffic-Aware Multi-View Video Stream Adaptation for Teleoperated Driving, Markus Hofbauer, Christopher B. Kuhn, Mariem, Goran Petrovic, Eckehard Steinbach; VTC2022-Spring* [[PDF](https://www.researchgate.net/publication/tbd)]

```tex
@inproceedings{hofbauer_2022,
    title = {Traffic-Aware Multi-View Video Stream Adaptation for Teleoperated Driving},
    booktitle = {2022 IEEE 95th Vehicular Technology Conference (VTC2022-Spring)},
    publisher = {IEEE},
    address = {Helsinki, Finland},
    author = {Hofbauer, Markus and Kuhn, Christopher B. and Khlifi, Mariem and Petrovic, Goran and Steinbach, Eckehard},
    month = {June},
    year = {2022},
    pages = {1--7},
}
```

## Setup

This so far has been tested on

| OS  | ROS Version |
| --- | ----------- |
| Ubuntu 20.04 | Noetic |

1. Download [CARLA](https://github.com/carla-simulator/carla/releases/latest)
1. Install [ROS](http://wiki.ros.org/ROS/Installation) and [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-catkin-tools)
1. Create a workspace with e.g. `mkdir -p ~/catkin_ws_teleop/src && cd ~/catkin_ws_teleop/src`
1. Clone this repository into the workspace's `src` folder with `git clone --recurse-submodules https://github.com/hofbi/tamva`
1. Run the install script: `./install.sh`
1. Build the workspace: `catkin build`
1. Source your workspace `source ~/catkin_ws_teleop/devel/setup.<your_shell>`

## Run

See the main module for running the application: [view_adaptation](view_adaptation/README.md#Run)

## Development

To install the additional tools required for the development, call

```shell
python3 -m pip install -r requirements.txt
sudo apt install -y clang-format clang-tidy-10
sudo snap install shfmt
```

### pre-commit git hooks

We use [pre-commit](https://pre-commit.com/) to manage our git pre-commit hooks.
`pre-commit` is automatically installed from `requirements.txt`.
To set it up, call

```sh
git config --unset-all core.hooksPath  # may fail if you don't have any hooks set, but that's ok
pre-commit install --overwrite
```

#### Usage

With `pre-commit`, you don't use your linters/formatters directly anymore, but through `pre-commit`:

```sh
pre-commit run --file path/to/file1.cpp tools/second_file.py  # run on specific file(s)
pre-commit run --all-files  # run on all files tracked by git
pre-commit run --from-ref origin/master --to-ref HEAD  # run on all files changed on current branch, compared to master
pre-commit run <hook_id> --file <path_to_file>  # run specific hook on specific file
```
