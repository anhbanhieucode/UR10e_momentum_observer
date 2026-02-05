# UR10e Momentum Observer

This project implements a momentum observer algorithm for the UR10e robot to detect collisions and estimate disturbance torques in industrial robot control.

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
- [References](#references)
- [Contributing](#contributing)
- [License](#license)

## Introduction

Momentum Observer is a state estimation technique widely used in physical Human-Robot Interaction (pHRI). This project implements momentum observer and collision detection algorithms for the UR10e robot using MuJoCo physics simulation. The project provides:

- **MuJoCo Simulation**: High-fidelity physics simulation of UR10e robot
- **Disturbance Torque Estimation**: Detect external forces acting on the robot using momentum observer
- **PID Control**: Joint-level trajectory tracking control
- **Visualization**: Real-time plotting of trajectories, torques, and disturbances
- **Dynamic Model**: Utilize UR10e dynamics for accurate computation

## System Requirements

- **Operating System**: Ubuntu 20.04+, or macOS 10.15+
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

A virtual environment isolates project dependencies and prevents conflicts with other Python packages on your system.

#### Clone the repository:

```bash
git clone https://github.com/anhbanhieucode/UR10e_momentum_observer.git
cd UR10e_momentum_observer
```

#### Create virtual environment:

**On Linux/macOS:**
```bash
python3 -m venv venv
```

#### Activate virtual environment:

**On Windows (Command Prompt):**
```bash
venv\Scripts\activate
```

**On Windows (PowerShell):**
```bash
venv\Scripts\Activate.ps1
```

**On Linux/macOS:**
```bash
source venv/bin/activate
```

After activation, you will see `(venv)` at the beginning of your command prompt.

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

```bash
python main.py
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

You can modify these parameters in `main.py`:

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


## Troubleshooting

### Common Errors:

**1. ModuleNotFoundError: No module named 'mujoco'**
```bash
# Make sure virtual environment is activated
source venv/bin/activate  # Linux/macOS
venv\Scripts\activate     # Windows

# Install MuJoCo
pip install mujoco
```

**2. FileNotFoundError: scene.xml not found**
```bash
# Ensure the UR10e model file exists at:
# universal_robots_ur10e/scene.xml
# Check the xml_path in main.py matches your directory structure
```

**3. Import Error with local modules (controllers, visualizer, etc.)**
```bash
# Make sure you're running from the project root directory
cd UR10e_momentum_observer
python main.py
```

**4. MuJoCo rendering issues on Linux**
```bash
# Install required graphics libraries
sudo apt-get install libgl1-mesa-glx libglew-dev
```

**5. NumPy compatibility issues**
```bash
# Update NumPy to a compatible version
pip install --upgrade numpy
```

### Deactivate virtual environment:

```bash
deactivate
```



## License

This project is distributed under the MIT License. See `LICENSE` file for more details.

## Contact

For any questions, please create an issue on GitHub or contact via email.

---

**Note**: This project uses MuJoCo for physics simulation. To connect with a real UR10e robot, additional libraries and communication protocols (such as `python-urx` or ROS) would need to be integrated. Always ensure safety measures when working with real robots.
