# RoboND - Go Chase It

---

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
