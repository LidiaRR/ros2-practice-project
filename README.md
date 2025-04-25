# TurtleBot Project

This project is a ROS 2 practice project using `turtlesim`. <b>It follows the tutorial at <http://wiki.ros.org/turtlesim/Tutorials>.</b>

## Source files

- `src/robot_cleaner/src/move.cpp`: It asks the user for a speed, a distance and whether the turtle should move
   forward or backward and then moves the turtle accordingly in a straight line.
- `src/robot_cleaner/src/rotate.cpp`: It asks the user for an angular velocity, an angle and whether the turtle should move
   clockwise or counterclockwise and then rotates the turtle accordingly.
- `src/robot_cleaner/src/gotogoal.cpp`: It asks the user for the coordinates of a position in the 2D map
    and a tolerance value and then moves the turtle to the goal.

## Execution

While running `turtlesim`, write the following code in a new terminal while you are in your ROS 2 workspace:

```bash
colcon build --packages-select robot_cleaner
source install/setup.bash
ros2 run robot_cleaner <node>
```

You should replace `<node>` with one of the possible nodes (`move`, `rotate` or `goto`).
