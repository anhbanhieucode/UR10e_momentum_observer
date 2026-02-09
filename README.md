# UR10e Momentum Observer
**Note:** New version with scenario_2 is updated on branch scenario_2
This project implements a momentum observer algorithm for the UR10e robot to detect collisions and estimate disturbance torques in industrial robot control.

Full report can be found here:

[6DOF_dynamics_ver4.pdf](https://github.com/user-attachments/files/25101964/6DOF_dynamics_ver4.pdf)

## Table of Contents

- [Introduction](#introduction)
- [System Requirements](#system-requirements)
- [Installation](#installation)
  - [1. Python Installation](#1-python-installation)
  - [2. Creating Virtual Environment](#2-creating-virtual-environment)
  - [3. Installing Dependencies](#3-installing-dependencies)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Theory](#theory)
- [License](#license)

## Introduction

Momentum Observer is a state estimation technique using difference in momentum with estimated dynamic models of the system. This project implements momentum observer and collision detection algorithms for the UR10e robot using MuJoCo physics simulation. The project provides:

- **MuJoCo Simulation**: High-fidelity physics simulation of UR10e robot
- **Disturbance Torque Estimation**: Detect external forces acting on the robot using momentum observer
- **PID Control**: Joint-level trajectory tracking control
- **Visualization**: Real-time plotting of trajectories, torques, and disturbances
- **Dynamic Model**: Utilize UR10e dynamics for accurate computation

## Preview result
The simulation will be divided into two scenarios for collision detection:

Scenario 1: The experiment for collision detection is to detect external force applied on a random joint (in this case joint2 ) 

Scenario 2: Using  Fault Detection and Isolation observer for emergency stop after an external obstacle collided with the robot 

The end effector of UR10e robot is setup to follow a circular trajectory while performing each experiment

**Scenario 1: Collision detection**

Video demo:

https://github.com/user-attachments/assets/d82cd402-5e8b-4610-a82b-3333e68670f7


Circular trajectory tracking:


![circular trajectory tracking](https://github.com/user-attachments/assets/a344ed86-e396-4000-9071-9599c0bc1d51)

Joint space trajectory tracking


![trajectory_tracking](https://github.com/user-attachments/assets/fede15b5-970d-447b-9d91-6501f0807c47)

Momentum observer:


With 20Nm external torque applied to the Joint 2: Elbow 


![Momentum_observer](https://github.com/user-attachments/assets/bc0d3566-6b2a-45e6-becd-2d1c52e6752d)

**Scenario 2: FDI observer:**

Video demo:

https://github.com/user-attachments/assets/3d6e3b9b-97c6-4082-83a2-4c50aaf57b1e


A red sphere is dropped to the robot, caused collision. After residual momentum was above a threshold, the robot stopped immediately.

![Residual_observer](https://github.com/user-attachments/assets/03c5f789-5ccb-4bab-be2a-f548b3254177)

Joint trajectory at collision

![Joint_position_when_collision_happend](https://github.com/user-attachments/assets/fbe30a96-9683-4f2f-a7a0-00e338f8fb59)


## System Requirements

- **Operating System**: Ubuntu 20.04+
- **Python**: Version 3.8 or higher (recommended Python 3.9 or 3.10)
- **RAM**: Minimum 4GB (8GB recommended for smooth MuJoCo simulation)
- **Storage**: At least 2GB free space
- **Graphics**: OpenGL-compatible graphics card (for MuJoCo visualization)

## Installation

### 1. Python Installation

#### On Ubuntu/Linux:

```bash
sudo apt update
sudo apt install python3.10 python3.10-venv python3-pip
```

### 2. Creating Virtual Environment


#### Clone the repository:

```bash
git clone https://github.com/anhbanhieucode/UR10e_momentum_observer.git
cd UR10e_momentum_observer
```

#### Create virtual environment:

**On Linux:**
```bash
python3 -m venv venv
```

#### Activate virtual environment:

**On Linux:**
```bash
source venv/bin/activate
```

After activation,  `(venv)` will appear at the beginning of the command prompt.

### 3. Installing Dependencies

After activating the virtual environment, install the required dependencies:

#### Install MuJoCo

MuJoCo (Multi-Joint dynamics with Contact) is the physics engine used for simulation.

```bash
pip install mujoco
```

#### Install other dependencies:

```bash
pip install numpy matplotlib
```


#### Main Libraries:


**Note**: The code also uses custom local modules:
- `controllers` (PIDController, FDI)
- `visualizer` (plot_comparison, plot_IK, plot_trajectory_comparison)
- `ur10e_dynamics` (get_dynamics)
- `IK_ur10e` (TrajectoryGenerator)
- `error` (inject_force)

**Install required dependencies** file with these contents:

```txt
mujoco>=3.0.0
numpy>=1.24.0
matplotlib>=3.7.0
```

## Project Structure

```
UR10e_momentum_observer/
|
├── UR10_MATLAB                  #File matlab used to calculate M , C , G and Inverse kinematic of UR10e 6DOF robot
│
├── __pycache__/                 # Python cache directory (auto-generated)
│
├── universal_robots_ur10e/      # UR10e robot model directory
│   ├── assets/                  # Model assets
│   ├── CHANGELOG.md            # Model changelog
│   ├── LICENSE                 # Model license
│   ├── README.md               # Model documentation
│   ├── compare.py              # Model comparison utilities
│   ├── gen_urdf.py             # URDF generation script
│   ├── scene.xml               # Main MuJoCo scene file
│   ├── ur10e.png               # Robot image
│   ├── ur10e.urdf              # URDF model file
│   ├── ur10e.xml               # XML model file
│   └── ur10e_synced.urdf       # Synchronized URDF file
│
├── IK_ur10e.py                  # Inverse Kinematics and Trajectory Generator
├── controllers.py               # PID Controller and FDI implementations
├── error.py                     # Force injection for collision simulation
├── main.py                      # Main program file (entry point)
├── ur10e_dynamics.py            # UR10e robot dynamics model
├── visualizer.py                # Plotting and visualization functions
│
├── requirements.txt             # Dependencies list (create this)
└── README.md                    # This documentation
```

### Module Descriptions

- **main.py**: Entry point of the program. Loads MuJoCo model, initializes controllers, runs simulation
- **controllers.py**: Contains `PIDController` class for joint-level control and `FDI` (Fault Detection and Isolation) for collision detection
- **visualizer.py**: Provides plotting functions including `plot_comparison`, `plot_IK`, and `plot_trajectory_comparison` 
- **ur10e_dynamics.py**: Implements `get_dynamics()` function that returns the dynamic model (M, C, G matrices) of UR10e
- **IK_ur10e.py**: Contains `TrajectoryGenerator` class for creating smooth joint trajectories
- **error.py**: Includes `inject_force()` function to simulate external disturbances/collisions during simulation
- **universal_robots_ur10e/**: Contains the complete UR10e robot model files for MuJoCo simulation

## Usage

### Prerequisites

The UR10e MuJoCo model is already included in the repository under:
```
universal_robots_ur10e/scene.xml
```

Make sure all the required Python modules are in the same directory as `main.py`:
- `controllers.py`
- `visualizer.py`
- `ur10e_dynamics.py`
- `IK_ur10e.py`
- `error.py`

### Basic execution:
To run scenario 1:
```bash
python3 main.py
```
To run scenario 2:
```bash
python3 main_FDI.py
```

### Program Flow

The `main.py` file includes:

1. **Collision Configuration**: Define collision parameters for specific joints
   ```python
   collision_config = {
       'joint': 1,        # Joint number to inject collision
       'start': 3.0,      # Start time (seconds)
       'end': 5.0,        # End time (seconds)
       'mag': 0.0         # Magnitude of force (Nm)
   }
   ```

2. **MuJoCo Model Loading**: Load the UR10e robot model
   ```python
   model = mujoco.MjModel.from_xml_path(xml_path)
   data = mujoco.MjData(model)
   ```

3. **PID Controller Setup**: Configure PID gains for each joint
   ```python
   pid = PIDController(
       kp=[100, 100, 100, 800, 900, 1000],
       ki=[1, 1, 1, 1, 1, 1],
       kd=[20, 20, 20, 20, 20, 20],
       dt=DT
   )
   ```

### Key Parameters

Modify parameters in `main.py`:

- **PID Gains**: 
  - `kp`: Proportional gains for each joint
  - `ki`: Integral gains for each joint
  - `kd`: Derivative gains for each joint

- **Collision Settings**:
  - `joint`: Which joint to simulate collision (0-5)
  - `start`: When collision starts (seconds)
  - `end`: When collision ends (seconds)
  - `mag`: Force magnitude (Nm)

### Running Simulation

The simulation will:
1. Load the UR10e model
2. Initialize controllers and observers
3. Generate trajectory
4. Run MuJoCo simulation
5. Apply collision forces if configured
6. Plot results (joint positions, torques, disturbances)

### Visualization

The program automatically generates comparison plots showing:
- Desired vs Actual joint trajectories
- Control torques
- Disturbance detection
- Collision events

## Theory

### Momentum Observer

The momentum observer estimates disturbance torques based on robot dynamics equation:

```
τ = M(q)q̈ + C(q,q̇)q̇ + g(q) + τ_ext
```

Where:
- `τ`: Measured joint torque
- `M(q)`: Inertia matrix
- `C(q,q̇)`: Coriolis and centrifugal matrix
- `g(q)`: Gravity vector
- `τ_ext`: External disturbance torque (to be estimated)

### Observer Equation

```
p̂ = M(q)q̇ + K_o(p̂ - p)
τ̂_ext = p̂˙ - C(q,q̇)q̇ - g(q) + τ
```

Where:
- `p̂`: Estimated momentum
- `K_o`: Observer gain matrix (typically β*I)
- `β`: Tuning parameter (typically 5 to 30)


### Visualization

The program automatically generates plots:

1. **Disturbance Torque vs Time**: Display disturbance torque over time and show the performance of observer
2. **Joint Positions**: Position of all joints to follow predefined trajectory


### Deactivate virtual environment:

```bash
deactivate
```



## License

This project is distributed under the MIT License. See `LICENSE` file for more details.

## Contact

For any questions, please contact via email:
hieuvanminhtran244@gmail.com

**Note**: This project uses MuJoCo for physics simulation. To connect with a real UR10e robot, additional libraries and communication protocols (such as `python-urx` or ROS) would need to be integrated. Always ensure safety measures when working with real robots.
