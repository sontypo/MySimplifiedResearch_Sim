---

# Application of Social Force Model in Socially Interactive Robotics

This repository contains a Python-based simulation of a mobile robot navigating a hallway environment while avoiding humans and walls. The robot uses a **social force model** to calculate forces such as desired force (`F_des`), social force (`F_soc`), boundary force (`F_bound`), and total force (`F_total`). The simulation is visualized in real-time using Matplotlib.

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Dependencies](#dependencies)
4. [Installation](#installation)
5. [Usage](#usage)
6. [File Structure](#file-structure)
7. [Simulation Details](#simulation-details)
8. [Contributing](#contributing)
9. [License](#license)

---

## Overview

The simulation consists of three main components:

1. **Environment**: A hallway with walls and humans moving along predefined trajectories.
2. **Robot Physics**: A dynamic model of the robot that calculates forces and updates its position and velocity.
3. **Visualization**: Real-time visualization of the robot, humans, and forces using Matplotlib.

The robot navigates towards a goal while avoiding collisions with humans and walls. The simulation stops when the robot reaches the goal.

---

## Features

- **Robot Dynamics**:
  - The robot's motion is governed by a dynamic model that accounts for forces such as `F_des`, `F_soc`, and `F_bound`.
  - The robot stops when it reaches the goal.

- **Human Interaction**:
  - Humans move along predefined trajectories.
  - The robot only reacts to humans within its **field of view (FOV)**.

- **Force Visualization**:
  - Real-time plots of the `x` and `y` components of `F_des`, `F_soc`, `F_bound`, and `F_total`.

- **Customizable Parameters**:
  - Hallway dimensions, robot mass, maximum velocity, force parameters, and more.

---

## Dependencies

The simulation requires the following Python libraries:

- `numpy`
- `matplotlib`

You can install these dependencies using `pip`:

```bash
pip install numpy matplotlib
```

---

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/sontypo/MySimplifiedResearch_Sim.git
   cd MySimplifiedResearch_Sim/
   ```

2. Install the dependencies:

   ```bash
   pip install -r requirements.txt
   ```

---

## Usage

Run the simulation using the following command:

```bash
python main.py
```

### Simulation Controls

- The simulation runs in real-time with a time step of `0.01` seconds.
- The robot will move towards the goal at `[12.5, 0]` and stop when it is within `0.1 m` of the goal.
- The visualization updates in real-time, showing the robot, humans, and force plots.

---

## File Structure

The repository has the following structure:

```
robot-simulation/
├── environment.py       # Defines the hallway environment and visualization
├── robot_dynamics.py     # Implements the robot's dynamics and force calculations
├── main.py              # Runs the simulation
├── README.md            # Project documentation
└── requirements.txt     # List of dependencies
```

---

Here’s the updated **Simulation Details** section of the `README.md` file, now including explanations of the functions inside the `robot_physics.py` script.

---

## Simulation Details

### Robot Dynamics

The robot's motion is governed by the following forces:

1. **Desired Force (`F_des`)**:
   - Drives the robot towards its goal.

2. **Social Force (`F_soc`)**:
   - Repels the robot from humans within its FOV.

3. **Boundary Force (`F_bound`)**:
   - Repels the robot from the walls.

4. **Total Force (`F_total`)**:
   - The sum of `F_des`, `F_soc`, and `F_bound`.

---

### Functions in `robot_physics.py`

The `robot_dynamics.py` script contains the following key functions:

#### 1. `__init__(self, ...)`
- **Purpose**: Initializes the robot's state and parameters.
- **Parameters**:
  - `total_mass`: Mass of the robot.
  - `v_max_robot`: Maximum velocity of the robot.
  - `tau_d`: Time constant for desired force calculation.
  - `sigma`, `Lambda`, `alpha`, `beta`, `alpha_bound`, `beta_bound`: Force parameters.
  - `boundary_dis`: Width of the hallway.
  - `cam2com`: Distance from the camera to the center of mass.
  - `fov_angle`: Field of view angle (in radians).
  - `fov_range`: Maximum detection range for humans.
- **Description**:
  - Sets up the robot's initial position, velocity, and orientation.
  - Defines parameters for force calculations and dynamics.

#### 2. `gamma_function(self, Lambda, cos_phi_ij)`
- **Purpose**: Computes the anisotropic behavior factor \( \Gamma \).
- **Parameters**:
  - `Lambda`: Anisotropy parameter.
  - `cos_phi_ij`: Cosine of the angle between the robot's velocity and the vector to the human.
- **Returns**:
  - The anisotropic behavior factor \( \Gamma \).

#### 3. `calculate_distance(self, pos1, pos2)`
- **Purpose**: Calculates the Euclidean distance between two points.
- **Parameters**:
  - `pos1`, `pos2`: 2D positions (numpy arrays).
- **Returns**:
  - The distance between `pos1` and `pos2`.

#### 4. `is_in_fov(self, human_position)`
- **Purpose**: Checks if a human is within the robot's field of view (FOV).
- **Parameters**:
  - `human_position`: Position of the human.
- **Returns**:
  - `True` if the human is within the FOV, otherwise `False`.

#### 5. `social_force_calc(self, human_positions, human_velocities)`
- **Purpose**: Calculates the social force (`F_soc`) based on humans within the FOV.
- **Parameters**:
  - `human_positions`: List of human positions.
  - `human_velocities`: List of human velocities.
- **Description**:
  - Iterates over humans within the FOV and computes the social force using the social force model.

#### 6. `boundary_force_calc(self)`
- **Purpose**: Calculates the boundary force (`F_bound`) based on distances to the walls.
- **Description**:
  - Computes the repulsive force from the left and right walls and sums them to get `F_bound`.

#### 7. `update_dynamic(self, Fm, Fn, v_x, omega_z, dt)`
- **Purpose**: Updates the robot's dynamics based on applied forces.
- **Parameters**:
  - `Fm`, `Fn`: Forces in the x and y directions.
  - `v_x`, `omega_z`: Current linear and angular velocities.
  - `dt`: Time step.
- **Returns**:
  - The updated linear and angular velocities.

#### 8. `update(self, human_positions, human_velocities, dt)`
- **Purpose**: Updates the robot's state (position, velocity, and forces).
- **Parameters**:
  - `human_positions`: List of human positions.
  - `human_velocities`: List of human velocities.
  - `dt`: Time step.
- **Returns**:
  - The updated robot position, total force, and force components (`F_des`, `F_soc`, `F_bound`, `F_total`).

---

### Visualization

The simulation visualization consists of two sections:

1. **Left Side**:
   - The robot (orange circle) and humans (blue circles) in the hallway.
   - A green arrow represents the total force acting on the robot.

2. **Right Side**:
   - Four subplots showing the `x` and `y` components of `F_des`, `F_soc`, `F_bound`, and `F_total` over time.

---

## Contributing

Contributions are welcome! If you'd like to contribute, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Commit your changes.
4. Submit a pull request.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- This simulation is inspired by social force models used in pedestrian and robot navigation.
- Thanks to the Matplotlib community for providing excellent visualization tools.

---
