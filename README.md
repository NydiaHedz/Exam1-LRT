# Exam1-LRT

This repository contains two exercises implemented using ROS (Robot Operating System) to control turtles in the turtlesim simulator.

## Exercise 1: Random Turtle Spawning
This program generates 5 additional turtles at random positions within the Turtlesim workspace. Each turtle is placed at random (x,y) coordinates with a random orientation. The program uses Turtlesim's Spawn service to create the new instances.

The developed code can be seen in: [turtle_five.py](https://github.com/NydiaHedz/Exam1-LRT/blob/main/Exam1/src/exam_1/exercise_1/turtle_five.py)

### Key Features
1. **Deletion of the Default Turtle**:
   - The developed program attempts to delete the default turtle (`turtle1`) using the `/kill` service.
   - If the turtle does not exist, a message is displayed in the console.

   ```python
   try:
       kill_turtle('turtle1')  # Delete the first turtle if it exists
       print("Turtle 1 deleted.")
   except rospy.ServiceException:
       print("Turtle 1 does not exist, cannot delete.")
   ```

2. **Spawning Turtles at Random Positions**:
   - The script generates 5 new turtles with unique names.
   - The `(x, y)` coordinates for each turtle are randomly generated within the range of 1.0 to 10.0.

   ```python
   for i in range(5):
       x = random.uniform(1.0, 10.0)  # Random x coordinate
       y = random.uniform(1.0, 10.0)  # Random y coordinate
       spawn_turtle(x, y, 0.0, f'turtle{i+1}')  # Spawn turtle with unique name
       print(f"Generated turtle {i+1} at position ({x:.2f}, {y:.2f}).")
   ```

3. **Use of ROS Services**:
   - The script uses the `/spawn` and `/kill` services provided by `turtlesim` to create and delete turtles, respectively.
   - It waits for these services to become available before proceeding.

   ```python
   rospy.wait_for_service('/spawn')
   rospy.wait_for_service('/kill')
   spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
   kill_turtle = rospy.ServiceProxy('/kill', Kill)
   ```

## Exercise 1: Random Turtle Spawning
This program implements a system for drawing geometric shapes using the Turtlesim turtle. The main features are:

* Selection menu with pairs of shapes:
   * Rhombus and Pentagon
   * Triangle and Isosceles Trapezoid
   * Square and Rhomboid
   * Rectangle and Trapezoid
* The user can define the initial position of the turtle
* The system verifies that all corners of the shape are within the workspace
* Console display of the selected shape information and coordinates of its corners
* The turtle draws the shape using a Proportional controller for movement
* Upon completion, the program notifies the user and waits for a key press to return to the main menu
* Includes an option to exit the program

