# Blender-ROS

## Stream joint angle positions from a Blender skeleton to a ROS node for controlling a robot controller.

### Setup (as tested):
  * Ubuntu 14.04
  * Blender 2.7x
  * ROS Indigo
  * `arbotix_ros` package
  * Robot controller:
    * Arbotix-M board
    * Dynamixel AX-12A servos (on HR-OS1 Endoskeleton Kit)
  * Redis

### Components:
  * `stream.py` -- Python code to run inside Blender.
  * `blender_ros_subscriber_node.py` -- ROS node.

### How it works:
  1. When `stream.py` script is run inside Blender, it will publish a (user-selected) set of joint angles to a Redis topic (e.g. 'test1', 'test2').
  2. The `blender_ros_subscriber_node.py` script listens to the Redis topic, do necessary transformations (still in progress), and publish the joint angle values to the ROS topic for the robot controller (`arbotix_ros`)

### Quickstart:

#### Robot controller setup
  1. Connect the servos to the Arbotix-M board
  2. Power the Arbotix-M board and connect to the computer via USB (requires an FTDI converter)

#### ROS setup
Everything here assumes ROS Indigo is installed and configured, including the Catkin workspace. If not, please follow the tutorials [here](wiki.ros.org/ROS/Tutorials) (Also make sure to follow the Indigo tutorials.)
  1. Clone this ROS package into the Catkin workspace:

    ```
    $ cd catkin_ws/src
    $ git clone https://github.com/msunardi/blender_ros.git
    ```

  2. (Assuming `arbotix_ros` is installed\*) Make sure the `blender_ros.launch` file points the `arbotix_driver` node to the `launch/arbotix_config_blender.yaml` file. This file defines the robot controller configuration, e.g.: servo IDs, servo names, limits, max/min speeds, etc.
    * Note: Also make sure the joint names in the `.yaml` file match the joint names in `blender_ros_subscrbier_node.py`.
  3. Run the subscriber node using the launch file

    ```
    $ roslaunch blender_ros blender_ros.launch
    ```


#### Blender setup
  1. Launch Blender
  2. In the Blender Python console, copy and paste the `stream.py`
    * Make sure the Python Redis package is installed - e.g. `$ sudo pip install redis`
  3. Click on the 'Run script' button to execute the script.
  4. Load an animation file/sequence and play it.
    * Note: as tested, the animation used was from a .bvh motion capture file and the joint angles used was from a skeleton/rig. Rename the objects in the `stream.py` to match the names of the object/skeleton/rig in the animation file.

### Video
<a href="https://www.youtube.com/watch?feature=player_embedded&v=kVInCRP2rSg" target="_blank"><img src="http://img.youtube.com/vi/kVInCRP2rSg/0.jpg" alt="Playing Motion Capture data on HR-OS1 robot (Jimmy)" width="240" height="180" border="10" /></a>
