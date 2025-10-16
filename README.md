# FAROL 2
This repository is home to the code stack for underwater marine vehicles (both AUVs and ROVs) used by the DSOR-ISR (Dynamical System and Ocean Robotics - Institute for Systems and Robotics) Team. It contains the base of the Guidance, Navigation and Control algorithms found in DSOR marine vehicles, such as the Medusa AUVs, BlueROV2, DELFIM Catamaran and SLOCUM Gliders.

## Disclaimer

This code stack is the product of the work of many people that passed through the DSOR-ISR organisation. It suffered countless iterations through the years. While its essence survived ever since, the same can't be said about its contribution history. The last version is available [here](https://github.com/dsor-isr/farol).
The current version is built using ROS2 Jazzy, aiming to migrate the previous one from ROS1 Noetic.

## Requirements

- Ubuntu 24.04 LTS
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Python 3.12.3

## Installation

Create a colcon workspace, as such:

```bash
mkdir -p ~/colcon_ws_sim/src
```

Clone this repository and its submodules to the colcon workspace:

```bash
cd ~/colcon_ws_sim/src
git clone --recurse-submodules git@github.com:dsor-isr/farol2.git
```

If you're planning on using FAROL 2 in your own repo, consider cloning FAROL 2 as a submodule instead:

```bash
git submodule add --recursive git@github.com:dsor-isr/farol2.git
```

## Install libraries

```
sudo apt install libgeographiclib-dev
```

## Using FAROL 2 QOL scripts and alias

In order to use FAROL 2 QOL scripts and alias, the following lines should be added to the `~/.bashrc` file. **Note that** these lines already add default ROS2 setup, including the `colcon_cd` command. **Remember to** set `$COLCON_HOME` to the directory where the colcon workspace is created.

```bash
#### FAROL 2 CONFIGURATIONS ####
# Create a file to store the latest colcon workspace (if it does not exist) and put in the first line the default name, i.e. colcon_ws
if [ ! -f ~/.colcon_ws_config ]; then touch ~/.colcon_ws_config && echo colcon_ws > ~/.colcon_ws_config ;fi

# Set the variable COLCON_WS with the workspace in the colcon_ws_config file
export COLCON_WS=$(head -n 1 ~/.colcon_ws_config)

# Function to update the default colcon workspace variable and store the last setting in the file
set_colcon_ws_function() {
    #set COLCON_WS according the an input parameter
    export COLCON_WS=colcon_ws_$1
    echo COLCON_WS = ${COLCON_WS}

    # save into a hidden file the catkin workspace setting
    echo $COLCON_WS > ~/.colcon_ws_config
    source ~/.bashrc
}

# ROS2
# source ros jazzy
source /opt/ros/jazzy/setup.bash

# source colcon workspace setup
export COLCON_HOME=${HOME}/dsor/ # change according to where the colcon workspace is created
export COLCON_ROOT=${COLCON_HOME}${COLCON_WS}
source ${COLCON_ROOT}/install/setup.bash

# create directory for saving ROS data
export ROSDATA_DIR=${COLCON_HOME}ROSData
if [ ! -d "$ROSDATA_DIR" ]; then
    mkdir -p "$ROSDATA_DIR"
fi

# set discovery range
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# set colcon_cd autocomplete
source /usr/share/colcon_cd/function/colcon_cd.sh
source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash
export _colcon_cd_root=${COLCON_ROOT}

# easy sourcing
alias S='source ${HOME}/.bashrc'
```

## Compiling

Source your colcon workspace dependent setup files, move to the colcon workspace, using `colcon_cd`, for example, and compile:

```bash
set_colcon_ws_function sim
colcon_cd
colcon build --symlink-install
```

<!-- ### Citation
If you use Farol 2 in a scientific publication, please cite:
```
TODO
```

### Documentation -->

### Active Developers
- Eduardo Cunha <eduardo.m.a.cunha@tecnico.ulisboa.pt>

### License
Farol is open-sourced under the MIT license. See the [LICENSE](LICENSE) file for details.

