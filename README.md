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

### Code Structure

The project is organized into the following components:

1. **Main Controller**:
   - Initializes the ROS node.
   - Manages the different drawing modules.
   - Provides a menu for user interaction.

2. **Drawing Modules**:
   - `CuadradoRomboide`: Draws a square and a rhomboid.
   - `RomboPentagono`: Draws a rhombus and a pentagon.
   - `TrianguloTrapecio`: Draws a triangle and a trapezoid.
   - `RectanguloTrapezoide`: Draws a rectangle and a trapezoid.
   - 
### Main Code
Made by: Nydia Hernández Bravo

The project consists of a main controller (`MainController`) that initializes and manages different modules for drawing geometric figures. Each module is responsible for drawing a specific pair of figures. The user interacts with the system through a text-based menu to select which figures to draw. This complete code can be seen in: [Main Code](https://github.com/NydiaHedz/Exam1-LRT/blob/main/Exam1/src/exam_1/exercise_2/main.py)

**How It Works**

The `MainController` class initializes the ROS node and the drawing modules. It provides a menu for the user to select which pair of figures to draw.

```python
class MainController:
    def __init__(self):
        # Initialize the ROS node only once
        rospy.init_node('main_controller', anonymous=True)

        # Initialize the modules
        self.cuadrado_romboide = CuadradoRomboide()
        self.rombo_pentagono = RomboPentagono()
        self.triangulo_trapecio = TrianguloTrapecio()
        self.rectangulo_trapezoide = RectanguloTrapezoide()

    def menu(self):
        """Main menu to select which module to run."""
        while not rospy.is_shutdown():
            print("\nMain Menu")
            print("1 -> Draw Square and Rhomboid")
            print("2 -> Draw Rhombus and Pentagon")
            print("3 -> Draw Triangle and Trapezoid")
            print("4 -> Draw Rectangle and Trapezoid")
            print("5 -> Exit")

            option = input("Select an option: ")

            if option == '1':
                self.cuadrado_romboide.menu()
            elif option == '2':
                self.rombo_pentagono.menu()
            elif option == '3':
                self.triangulo_trapecio.menu()
            elif option == '4':
                self.rectangulo_trapezoide.menu()
            elif option == '5':
                print("Exiting...")
                break
            else:
                print("Invalid option. Please try again.")
```

### Rhombus and Pentagon

Made by: Andoni Díaz Castellanos

#### 1. Introduction
This document provides a detailed analysis of a Python script designed to control a simulated turtle in a ROS (Robot Operating System) environment. The script enables the turtle to draw geometric figures based on user input. It covers the code structure, functionalities, and each component’s interaction with the ROS framework.

#### 2. Script Structure and Imports

The script starts with necessary Python imports and initialization of the ROS environment:

```python
#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
```

#### Explanation:
- **Shebang Line**: Ensures the script runs with Python 3.
- **`rospy`**: Used to interact with ROS.
- **`math`**: Provides functions for mathematical calculations.
- **`geometry_msgs.msg.Twist`**: Used for sending velocity commands to the turtle.
- **`turtlesim.msg.Pose`**: Used for receiving the turtle’s current pose.
- **`std_srvs.srv.Empty`**: May be used for services like resetting the simulation (not explicitly covered in the initial code but typical in ROS environments).

#### 3. Class Definition

#### 3.1 The `TurtleControl` Class

The class `TurtleControl` encapsulates all functionalities for controlling the turtle.

```python
class TurtleControl:
```

#### 3.2 Constructor and Initialization

```python
def __init__(self):
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
    self.current_pose = None
    self.rate = rospy.Rate(10)
    self.turtle_size = 0.5
```

#### Explanation:
- **`rospy.init_node`**: Initializes a ROS node named `turtle_keyboard_control` with a unique name due to `anonymous=True`.
- **`self.pub`**: Sets up a publisher on the `/turtle1/cmd_vel` topic to send velocity commands.
- **`self.pose_sub`**: Subscribes to the `/turtle1/pose` topic to update the turtle’s pose with each incoming message.
- **`self.current_pose`**: Initialized to `None` to store the turtle’s current pose.
- **`self.rate`**: Sets a loop rate of 10Hz.
- **`self.turtle_size`**: Assumes a turtle size of 0.5 units for collision and boundary checks.

#### 3.3 Update Pose

```python
def update_pose(self, data):
    self.current_pose = data
```

**Functionality**: This method updates the turtle’s current pose whenever a new `Pose` message is received from the ROS topic.

#### 3.4 Move to Point

```python
def move_to_point(self, target_x, target_y, kp_linear=1.0, kp_angular=2.0):
    while not rospy.is_shutdown():
        if self.current_pose is None:
            continue
        angle_to_target = math.atan2(target_y - self.current_pose.y, target_x - self.current_pose.x)
        distance = math.sqrt((target_x - self.current_pose.x) ** 2 + (target_y - self.current_pose.y) ** 2)
        angle_error = angle_to_target - self.current_pose.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_error) > 0.01:
            twist = Twist()
            twist.angular.z = kp_angular * angle_error
            self.pub.publish(twist)
        elif distance > 0.05:
            twist = Twist()
            twist.linear.x = kp_linear * distance
            twist.angular.z = kp_angular * angle_error
            self.pub.publish(twist)
        else:
            print(f"Reached corner at ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})")
            break
        self.rate.sleep()
    self.pub.publish(Twist())
    rospy.sleep(0.5)
```

**Explanation**: This method controls the turtle to move towards a specified point. It uses proportional control to adjust the turtle’s orientation and forward movement. It continues to adjust the turtle’s path until it is close enough to the target position, at which point it stops the turtle.


#### 4. Position Validation, User Interaction, and Figure Drawing

#### 4.1 Valid Position

```python
def valid_position(self, x, y):
    return x >= self.turtle_size and x <= 11 - self.turtle_size and \
           y >= self.turtle_size and y <= 11 - self.turtle_size
```

**Functionality**: Checks if the specified position is within valid boundaries, considering the turtle’s size.


#### 4.2 Request Position

```python
def request_position(self):
    while True:
        try:
            x, y = map(float, input("Enter start coordinates for drawing the figure (x, y): ").split())
            if self.valid_position(x, y):
                return x, y
            else:
                print("Invalid position. The figure would be out of the drawing area. Try again.")
        except ValueError:
            print("Invalid input. Please enter numeric coordinates.")
```

**Explanation**: Interacts with the user to get valid start coordinates for drawing. It continues to request input until valid coordinates are provided.

#### 4.3 Drawing Figures

```python
def draw_figure(self, figure_type):
    start_x, start_y = self.request_position()
    self.move_to_point(start_x, start_y)
    if figure_type == 'rhombus':
        # Code to draw a rhombus
    elif figure_type == 'pentagon':
        # Code to draw a pentagon
```

**Functionality**: Based on the user’s choice, this method directs the turtle to draw specified figures such as a rhombus or pentagon by moving to calculated positions.


#### 5. Main Method and Execution

#### 5.1 Main Method

```python
def run(self):
    while not rospy.is_shutdown():
        print("Press 'r' to draw rhombus, 'p' to draw pentagon, 'x' to exit")
        command = input().strip().lower()
        if command == 'x':
            break
        elif command == 'r':
            self.draw_figure('rhombus')
        elif command == 'p':
            self.draw_figure('pentagon')
```

**Explanation**: The main loop of the script, which processes user commands to control the turtle’s actions. It allows users to select the figure to draw or exit the program.

#### 5.2 Execution Block

```python
if __name__ == '__main__':
    try:
        turtle_control = TurtleControl()
        turtle_control.run()
    except rospy.ROSInterruptException:
        pass
```

**Explanation**: Ensures that the script runs only when executed directly. It starts the turtle control and handles any ROS interruption exceptions gracefully.
