# Nav2 - Butler Robot

This repository contains the ROS 2 packages for a butler robot that uses the Navigation2 stack to handle food delivery tasks. The robot can move between the kitchen, tables, and home position based on user input.

## Prerequisites

- **ROS 2 Foxy Fitzroy** or later.
- **Navigation2 Stack** installed.
- **RViz** for visualization.

## Packages

1. **basic_mobile_robot**: Contains the robot model, launch files, and configuration for the Navigation2 stack.
2. **butler_robot**: Contains the action server and client for handling food delivery tasks.

---

## Step-by-Step Implementation

### 1. Launch the Robot and Navigation2 Stack

Launch the robot model and the Navigation2 stack using the following command:

```bash
ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py
```

This will:
- Launch the robot model in Gazebo or RViz.
- Load the Navigation2 stack with all necessary parameter files.
- Start the navigation stack for autonomous movement.

---

### 2. Run the Action Server

In a new terminal, run the action server to handle food delivery tasks:

```bash
ros2 run butler_robot butler_action_server
```

#### Expected Output:
```
[INFO] [butler_action_server]: Butler Action Server started...
```

---

### 3. Run the Action Client

In another terminal, run the action client to send goals to the robot:

```bash
ros2 run butler_robot butler_action_client
```

The client will prompt you for input:
```
Enter the table number: table1
Require confirmation? (yes/no): no
Allow cancellation? (yes/no): no
Handle multiple orders? (yes/no): no
```

#### Expected Output:
```
[INFO] [butler_action_client]: Butler Action Client started...
[INFO] [butler_action_client]: Waiting for action server...
[INFO] [butler_action_client]: Sending goal...
[INFO] [butler_action_client]: Goal accepted :)
[INFO] [butler_action_client]: Received feedback: Current Location = Home, Status = Moving
[INFO] [butler_action_client]: Received feedback: Current Location = Kitchen, Status = Moving
[INFO] [butler_action_client]: Received feedback: Current Location = Table, Status = Moving
[INFO] [butler_action_client]: Received feedback: Current Location = Home, Status = Completed
[INFO] [butler_action_client]: Final Result: Success = True, Final Status = Completed
```

---

### 4. Robot Behavior

- The robot will:
  1. Move from the **Home** position to the **Kitchen**.
  2. Move from the **Kitchen** to the specified **Table**.
  3. Return to the **Home** position after completing the task.

---

## Terminal Output Example

### Action Server Output
```
[INFO] [butler_action_server]: Butler Action Server started...
[INFO] [butler_action_server]: Executing goal...
[INFO] [butler_action_server]: Moving to Kitchen...
[INFO] [butler_action_server]: Moving to Table...
[INFO] [butler_action_server]: Returning to Home...
```

### Action Client Output
```
[INFO] [butler_action_client]: Butler Action Client started...
[INFO] [butler_action_client]: Waiting for action server...
[INFO] [butler_action_client]: Sending goal...
[INFO] [butler_action_client]: Goal accepted :)
[INFO] [butler_action_client]: Received feedback: Current Location = Home, Status = Moving
[INFO] [butler_action_client]: Received feedback: Current Location = Kitchen, Status = Moving
[INFO] [butler_action_client]: Received feedback: Current Location = Table, Status = Moving
[INFO] [butler_action_client]: Received feedback: Current Location = Home, Status = Completed
[INFO] [butler_action_client]: Final Result: Success = True, Final Status = Completed
```

---

## Customization

- **Table Positions**: Update the table poses in the `butler_action_server.py` file.
- **Navigation Parameters**: Modify the Navigation2 parameters in the `basic_mobile_robot` package.

---

## Troubleshooting

- **Navigation Issues**: Ensure the map and robot localization are correctly configured in RViz.
- **Action Server/Client Errors**: Check for typos in the goal parameters or action definitions.

---

## Acknowledgments

- **ROS 2 Navigation2 Stack**: For providing the navigation framework.
- **Gazebo/RViz**: For simulation and visualization.

