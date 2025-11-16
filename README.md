![Isaac Lab](docs/source/_static/isaaclab.jpg)

---

# Isaac Lab

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.0.0-silver.svg)](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![License](https://img.shields.io/badge/license-BSD--3-yellow.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![License](https://img.shields.io/badge/license-Apache--2.0-yellow.svg)](https://opensource.org/license/apache-2-0)


**Isaac Lab** is a GPU-accelerated, open-source framework designed to unify and simplify robotics research workflows,
such as reinforcement learning, imitation learning, and motion planning. Built on [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html),
it combines fast and accurate physics and sensor simulation, making it an ideal choice for sim-to-real
transfer in robotics.

Isaac Lab provides developers with a range of essential features for accurate sensor simulation, such as RTX-based
cameras, LIDAR, or contact sensors. The framework's GPU acceleration enables users to run complex simulations and
computations faster, which is key for iterative processes like reinforcement learning and data-intensive tasks.
Moreover, Isaac Lab can run locally or be distributed across the cloud, offering flexibility for large-scale deployments.


## Key Features

Isaac Lab offers a comprehensive set of tools and environments designed to facilitate robot learning:

- **Robots**: A diverse collection of robots, from manipulators, quadrupeds, to humanoids, with 16 commonly available models.
- **Environments**: Ready-to-train implementations of more than 30 environments, which can be trained with popular reinforcement learning frameworks such as RSL RL, SKRL, RL Games, or Stable Baselines. We also support multi-agent reinforcement learning.
- **Physics**: Rigid bodies, articulated systems, deformable objects
- **Sensors**: RGB/depth/segmentation cameras, camera annotations, IMU, contact sensors, ray casters.


## Getting Started

### Prerequisites

- **Operating System**: Ubuntu 22.04 or Ubuntu 24.04
  - Ubuntu 24.04 is recommended if you plan to work with LiDAR and drones, as the PX4 repository with LiDAR support requires Ubuntu 24.04
- **ROS2 Distribution**: Jazzy
  - [Installation guide for ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

### Installation

1. **Install Isaac Sim and Isaac Lab**
   
   Follow the [official binaries installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html#isaaclab-binaries-installation).
   
   **Note**: You don't need to clone the IsaacLab repository separately since you already have this fork. Continue with the other instructions such as creating a virtual environment with uv and installing dependencies.

2. **Install Isaac Lab dependencies**
   
   From the root of the repository, run:
   ```bash
   ./isaaclab.sh --install  # or "./isaaclab.sh -i"
   ```
   
   When prompted, select: **directRL + PPO + AMP**

3. **Activate the virtual environment**
   
   ```bash
   source ./env_isaaclab/bin/activate  # or "source ./my_env/bin/activate"
   ```

### Running the Remote Sensing Script

Once installation is complete and the environment is activated, you can run the remote sensing drone script:

```bash
./isaaclab.sh -p scripts/tasks/remote_sensing_drone.py
```

This command should be executed from the root of the repository.

## License

The Isaac Lab framework is released under [BSD-3 License](LICENSE). The `isaaclab_mimic` extension and its
corresponding standalone scripts are released under [Apache 2.0](LICENSE-mimic). The license files of its
dependencies and assets are present in the [`docs/licenses`](docs/licenses) directory.

Note that Isaac Lab requires Isaac Sim, which includes components under proprietary licensing terms. Please see the [Isaac Sim license](docs/licenses/dependencies/isaacsim-license.txt) for information on Isaac Sim licensing.

Note that the `isaaclab_mimic` extension requires cuRobo, which has proprietary licensing terms that can be found in [`docs/licenses/dependencies/cuRobo-license.txt`](docs/licenses/dependencies/cuRobo-license.txt).

## Acknowledgement

Isaac Lab development initiated from the [Orbit](https://isaac-orbit.github.io/) framework. We would appreciate if
you would cite it in academic publications as well:

```
@article{mittal2023orbit,
   author={Mittal, Mayank and Yu, Calvin and Yu, Qinxi and Liu, Jingzhou and Rudin, Nikita and Hoeller, David and Yuan, Jia Lin and Singh, Ritvik and Guo, Yunrong and Mazhar, Hammad and Mandlekar, Ajay and Babich, Buck and State, Gavriel and Hutter, Marco and Garg, Animesh},
   journal={IEEE Robotics and Automation Letters},
   title={Orbit: A Unified Simulation Framework for Interactive Robot Learning Environments},
   year={2023},
   volume={8},
   number={6},
   pages={3740-3747},
   doi={10.1109/LRA.2023.3270034}
}
```
