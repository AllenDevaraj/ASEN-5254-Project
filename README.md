# Dual-Arm Cooperative Manipulation using TAMP

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)

![MoveIt 2](https://img.shields.io/badge/MoveIt-2-orange)

![Gazebo](https://img.shields.io/badge/Ignition-Fortress-red)

![Python](https://img.shields.io/badge/Python-3.10+-yellow)

**Author:** Allen Devaraj Augustin Ponraj  

**Affiliation:** University of Colorado Boulder, Dept. of Aerospace Engineering Sciences

## 📖 Overview

This project implements a **Task and Motion Planning (TAMP)** framework for a dual-arm robotic system. Two Franka Emika Panda arms coordinate to perform a multi-stage assembly task in a cluttered, unstructured environment.

The system autonomously:

1.  **Reasons symbolically:** Detects if objects are stacked and plans "unstacking" operations before attempting retrieval.

2.  **Plans motions:** Utilizes a hybrid approach combining **RRTConnect** (for global obstacle avoidance) and **Cartesian Interpolation** (for precise insertion).

3.  **Executes cooperation:** Performs a frame-based "peg-in-hole" insertion where one arm stabilizes the receptacle while the other inserts the peg, regardless of the receptacle's orientation.

---

## Demo
YouTube video link here...

## ⚙️ Prerequisites

* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)

* **Middleware:** ROS 2 Humble Hawksbill

* **Simulation:** Ignition Gazebo Fortress (6.17.0+)

* **Motion Planning:** MoveIt 2

### System Dependencies

Ensure you have the following ROS 2 packages installed:

```bash
sudo apt update
sudo apt install ros-humble-moveit \
                 ros-humble-ros-gz \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 python3-tk
```

-----

## 🛠️ Installation

1.  **Clone the Repository:**

    ```bash
    git clone https://github.com/AllenDevaraj/ASEN-5254-Project.git
    cd ASEN-5254-Project
    ```

2.  **Navigate to the ROS 2 Workspace:**

    ```bash
    cd ros2_ws
    ```

3.  **Install Python Dependencies:**

    ```bash
    pip3 install scipy numpy
    ```

4.  **Build the Workspace:**

    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

-----

## 🚀 Usage

The entire system (Simulation, MoveIt, Perception, and GUI) is orchestrated by a single launch file.

### 1\. Launch the System

```bash
ros2 launch pick_and_place dual_panda_single_group.launch.py
```

### 2\. Using the Control GUI

Once the simulation loads, the **Dual Panda Control** interface will appear.

  * **Live Monitor:** Check the right-hand panel to ensure "Ground Truth" poses are being received from Gazebo.

  * **Manual Control:** Use the "Panda 1" and "Panda 2" panels to test IK solutions (Move/Plan Only).

  * **Run the Task:**

    1.  Select **Global Controls** -\> `Move Both to Neutral`.

    2.  In the **Dual Arm Tasks** section:

          * Select the target objects (e.g., `red_hollow` and `red_solid`).

          * Click **"Insert Prep"**: This triggers the TAMP solver. The robot will automatically unstack blocking objects if necessary.

          * Click **"Insert"**: Executes the final Cartesian approach and insertion.

-----

## 🧩 Architecture

### Hierarchical TAMP Pipeline

  * **Perception:** `ros_gz_bridge` provides ground-truth poses to a `PerceptionOracle` node.

  * **Symbolic State:** The `TaskManager` maintains predicates (e.g., `ON(A, B)`, `HOLDING(Arm, A)`) to generate high-level plans.

  * **Geometric Planning:**

      * **Transport Phase:** RRTConnect (OMPL) for collision-free travel.

      * **Insertion Phase:** Jacobian-based Cartesian path planning for linear motion.

### Coordinate Frames

The insertion logic uses a relative frame chain:

`World -> Hollow_Object -> Insertion_Frame -> Solid_Object -> Gripper`

This allows the system to insert the peg successfully even if the holding arm is positioned at an arbitrary angle.

-----

## 📄 License

MIT License
