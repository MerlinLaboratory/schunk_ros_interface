# ROS2 Humble Wrapper for EGK Schunk Grippers

This repository serves as a ROS2 wrapper for controlling EGK Schunk grippers with Ethernet/IP connection. The repo leverages the [EIPScanner](https://github.com/nimbuscontrols/EIPScanner) repository to establish the necessary Ethernet/IP connection with the gripper. 

## Hardware Dependencies

**OS**: Linux Ubuntu 22.04
**Gripper**: EGK-40-E-I-P

Tested on Ubuntu 22.04 with ROS2 Humble. Soon a Docker file should be available to make it available for ROS1.
**While the code has been exclusivelly tested with the EGK40 model, support for all other EGK models is not excluded.**

## Features

- **Gripper State Logging**: Access real-time information about the gripper's state through a dedicated ROS topic.
- **Control Services**: Utilize ROS services to command various gripper actions, such as:
  - `jog_to`: Move the gripper to a specified position.
  - `release`: Release any grasped object.
  - `simple_grip`: Perform a simple gripping action.
  Additional information about each service can be found within the `schunk_interface` package.

## Repository Structure

The principal packages are briefly described in the following table:

| Package | Description |
| --- | --- |
| [Doc](Doc) | (Contains the necessary documentation to understand how to use the gripper and the .EDS file for Ethernet/IP Assembly communication. All the files can be found on the Schunk website.). |
| [schunk_hardware_interface](schunk_hardware_interface) |  Contains the actual code responsible for establishing the connection with the gripper and providing the necessary services and topics.  |
| [schunk_interface](schunk_interface) | Contains information about the messages and services used by this repository. |

## Usage

### Prerequisites
ROS2 Humble needs to be installed and configured in your computer.

The only other dependency that the repo requires is [EIPScanner](https://github.com/nimbuscontrols/EIPScanner). Just clone it and build it (it is not necessary to put this repo in your ROS workspace):
  ```
  git clone https://github.com/nimbuscontrols/EIPScanner.git
  mkdir build && cd build
  cmake ..
  cmake --build . --target install
  ```

  Finally, in the same terminal where you are going to launch the gripper node, run:
  ```
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
  ```
  otherwise place it inside your .bashrc file.

### Installation
1. Clone the repository into your ROS2 workspace:
    ```
    git clone https://github.com/MerlinLaboratory/schunk_ros_interface.git
    ```
2. Build the package using colcon:
    ```
    colcon build
    ```

### Usage
1. **Launch the Node**: Launch the node to start communication with the gripper.
    ```
    ros2 run ... ...
    ```

2. **Access Gripper State**: Subscribe to the gripper state topic to receive updates on the gripper's status.
    ```
    ros2 topic echo ...
    ```

3. **Control the Gripper**: Call the provided services to command the gripper's actions. For example:
    ```
    ros2 service call ... ...
    ```

## Future Development

In the future, a Docker implementation will be made available to simplify integration and improve compatibility with ROS1.