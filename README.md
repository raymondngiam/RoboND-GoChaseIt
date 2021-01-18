# RoboND - Go Chase It

---

### Implementation Details

**`my_robot` ROS packages** consists of

- A differential drive robot designed using Unified Robot Description Format (URDF)
- Robot consists of lidar and camera sensor.
- Gazebo plugins for the robot's differential drive, lidar, and camera

**`ball_chaser` ROS package** consists of

- `drive_bot` node

    - Provides a `ball_chaser/command_robot` service
    - Service accepts linear x and angular z velocities
    - Service publishes to the wheel joints

- `process_image` node

    - Subscribes to the robot's camera image feeds
    - A function to analyze the image and determine the presence and position of a white ball
    - Requests a service to drive the robot towards a white ball (when present)


### Dependencies

- ROS Kinetic
- CMake 2.8

### Build Instruction

1. Clone the repo. From the repo root directory, build using `catkin_make`

    ``` shell
    $ cd {repo_root}/catkin_ws
    $ catkin_make
    ```

2. Launch the `world.launch` from `my_robot` package.

    ``` shell
    $ cd {repo_root}/catkin_ws
    $ source devel/setup.bash
    $ roslaunch my_robot world.launch
    ```

3. In a new terminal tab, launch the `ball_chaser.launch` from the `ball_chaser` package.

    ``` shell
    $ cd {repo_root}/catkin_ws
    $ source devel/setup.bash
    $ roslaunch ball_chaser ball_chaser.launch
