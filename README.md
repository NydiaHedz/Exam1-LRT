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

### Square and Rhomboid

Made by: Claudia Fernanda Mayoral Sánchez

#### General Objective

The purpose of the code is to enable the creation and control of geometric shapes within the TurtleSim environment of ROS (Robot Operating System), with a focus on creating simple geometric shapes: **squares** and **rhomboids**. The user can choose only one of the two shapes to draw in the environment, providing the initial coordinates where they want the drawing to start.

This interactive program uses **proportional control** to move the turtle precisely along the coordinates of the selected shape’s vertices. This ensures that the figures are drawn accurately, and at the end of each drawing, the workspace is cleared, allowing for a clean environment for new drawings. Additionally, proportional control helps stabilize the turtle's movement, preventing erratic movements and providing smooth and controlled motion of the turtle in the workspace. However, since the proportional controller is not the most precise, we can still observe oscillations during the turtle's drawing, as the movement continuously adjusts based on position error.


#### General Flow of the Program

1. **Elimination of the Initial Turtle**:
   - The TurtleSim environment starts with an initial turtle located at coordinates `(5.5, 5.5)`. To avoid problems with the creation of new turtles and the management of multiple turtles, the code kills this initial turtle at the start of the program. This is achieved by calling the function `kill_initial_turtle()`, which uses the `/kill` ROS service to eliminate the default turtle.

2. **Selection of the Figure to Draw**:
   - Through an interactive menu, the user can choose only between two options:
     1. Draw a **Square**.
     2. Draw a **Rhomboid**.
   - After selecting an option, the user enters the initial coordinates where they want the drawing to start.

3. **Creating the Turtle for Drawing**:
   - Once the user enters the coordinates, a new turtle is created in the workspace at those coordinates, using the `/spawn` service. This turtle will be responsible for drawing the selected figure. This is achieved by calling the function `create_turtle()`, which uses the ROS `Spawn` service.

4. **Drawing the Figure**:
   - The turtle moves from vertex to vertex of the selected figure (square or rhomboid) using proportional control to ensure smooth and precise movement.

5. **Handling Coordinates Outside the Workspace**:
   - If the coordinates entered by the user for the figure are outside the valid workspace area (`11x11`), the program will notify the user that the figure does not fit within the boundaries and will prompt them to enter new valid coordinates. This is achieved through the `validar_espacio` function.

6. **Notification of Completion**:
   - After completing the drawing, the program informs the user that the figure has been drawn successfully and waits for the user to press any key to return to the menu.

7. **Elimination of the Turtle and Cleaning the Area**:
   - After the drawing is complete, and the user presses a key to return to the menu, the turtle that performed the drawing is eliminated using the `/kill` service. Additionally, the area is cleared using the `/clear` service, ensuring that the workspace is clean and ready for the next drawing.

8. **Returning to the Menu**:
   - After pressing a key, the program deletes the turtle and clears the area, returning to the menu so the user can select whether they want to draw another figure or exit the program.

---

#### Proportional Control in the Code

The **proportional control** is used to move the turtle in a controlled manner toward the target. This type of control adjusts the turtle's speed in proportion to the position error (the difference between the turtle's current position and the desired position).

- **`move_to_point` function**: This function allows the turtle to move from one point to another using proportional control. As the turtle approaches the desired position, the speed decreases, preventing the turtle from moving erratically.
- **Proportionality constant (`Kp`)**: This value adjusts the speed of the turtle. An appropriate value of `Kp` ensures that the turtle moves efficiently and without jerky movements.
- **Stability**: The use of proportional control ensures that the turtle does not move too quickly as it approaches the destination, which is essential for maintaining precision when tracing the figures.

This control is crucial for drawing geometric shapes accurately, allowing for smooth and controlled movements, ideal for tasks like the ones performed by this code.

#### Implementation Details

1. **User Interaction**:
   - The code presents an interactive menu where the user can choose between drawing a **square** or a **rhomboid**. After selecting an option, the user is asked to input the initial coordinates to start the drawing.

2. **Drawing of Figures**:
   - **Square**: A square with a side length of 2 units is drawn. The turtle moves between the 4 vertices of the square. Although the square has 4 vertices, the console displays 5 points because the first point corresponds to the initial coordinates, and the last point returns to the starting point, closing the figure.
   - **Rhomboid**: The rhomboid has a base of 3 units, a height of 2 units, and an angle of 30 degrees. The turtle moves between the 4 vertices of the rhomboid. Similar to the square, 5 points are displayed in the console for the same reason (the last point corresponds to the initial point to close the figure).

3. **Elimination of Turtles and Cleaning**:
   - After each drawing, the turtle used to perform the drawing is eliminated using the `/kill` service, ensuring that no turtles accumulate. Additionally, the workspace is cleared with the `/clear` service to remove any previous traces and leave the space ready for the next drawing.

4. **Avoiding Errors with Multiple Turtles**:
   - The code handles the creation and deletion of turtles efficiently, avoiding the creation of unnecessary turtles and ensuring that no turtles accumulate in the workspace, which could interfere with turtle control.

#### Menu Options

The interactive menu allows the user to choose from the following options:

1. **Draw Square**: The user provides the coordinates for the square, and the turtle draws the figure. After completing the drawing, the area is cleared, and the turtle is eliminated.
2. **Draw Rhomboid**: Similar to the square, the user provides the coordinates for the rhomboid, and the turtle draws the figure. After completing the drawing, the area is cleared, and the turtle is eliminated.
3. **Exit**: Exits the program.

After each figure, the code waits for the user to press a key to return to the menu, ensuring that the environment is clean and ready for the next drawing.

#### Conclusion

This code provides an interactive tool for creating geometric shapes in the TurtleSim environment of ROS. It uses **proportional control** to ensure that the turtle moves in a precise and controlled manner, drawing figures such as squares and rhomboids with accuracy. The code efficiently manages the creation and elimination of turtles, ensuring that no unnecessary turtles accumulate and that the workspace remains clean and organized. This provides a smooth interactive experience where the user can draw figures and return to the main menu without complications.

Additionally, the program ensures that the coordinates provided by the user are within the workspace, and if they are not, the program notifies the user and requests new coordinates. This prevents errors and ensures that the figures are always drawn within the available space.
---

### Rectangle and Trapezoid

Made by: Nydia Hernández Bravo

This ROS (Robot Operating System) node allows you to control a turtle in the `turtlesim` simulator to draw rectangles and trapezoids. The script provides a menu-driven interface for user interaction, enabling the user to spawn a turtle at specific coordinates, move it to draw shapes, and then clear the workspace. The complete code can be seen in: [rectangulo_trapezoide.py](https://github.com/NydiaHedz/Exam1-LRT/blob/main/Exam1/src/exam_1/exercise_2/rectangulo_trapezoide.py)

#### Usage

1. Start the `turtlesim` simulator:
   ```bash
   rosrun turtlesim turtlesim_node
   ```

2. Run the script:
   ```bash
   python3 RectanguloTrapezoide.py
   ```

3. Follow the menu prompts to:
   - Draw a rectangle.
   - Draw a trapezoid.
   - Return to the main menu.

4. After drawing a shape, the turtle will be killed, and the workspace will be cleared.

#### Functionality

The script provides the following features:
- **Kill the default turtle**: The initial turtle (`turtle1`) is killed to start with a clean workspace.
- **Spawn a new turtle**: A new turtle is created at user-specified coordinates.
- **Draw shapes**: The turtle can draw rectangles and trapezoids based on user input.
- **Proportional control**: The turtle moves to target positions using proportional control for smooth movement.
- **Workspace validation**: The script ensures that the shapes fit within the 11x11 unit workspace.
- **Clear workspace**: After drawing, the workspace is cleared, and the turtle is killed.

#### Code Structure

The script is organized into a class `RectanguloTrapezoide` with the following methods:

- **`__init__`**: Initializes the node, kills the default turtle, and sets up the workspace dimensions.
- **`actualizar_posicion`**: Callback to update the turtle's current position.
- **`suscribirse_posicion`**: Subscribes to the turtle's position topic.
- **`configurar_publicador`**: Configures the velocity publisher for the turtle.
- **`kill_initial_turtle`**: Kills the default turtle (`turtle1`).
- **`crear_tortuga`**: Spawns a new turtle at specified coordinates.
- **`mover_a_punto`**: Moves the turtle to a target position using proportional control.
- **`validar_espacio`**: Validates if the shape fits within the workspace.
- **`dibujar_rectangulo`**: Draws a rectangle.
- **`dibujar_trapezoide`**: Draws a trapezoid.
- **`matar_tortuga`**: Kills the turtle after drawing.
- **`limpiar_area`**: Clears the workspace.
- **`menu`**: Provides a menu for user interaction.

#### How It Works

1. When the script starts, it kills the default turtle (`turtle1`) to ensure a clean workspace.
2. The user is presented with a menu to choose between drawing a rectangle or a trapezoid.
3. The user provides the starting coordinates for the shape.
4. A new turtle (`turtle2`) is spawned at the specified coordinates.
5. The turtle moves to each corner of the shape using proportional control, ensuring smooth and accurate movement.
6. After drawing the shape, the turtle is killed, and the workspace is cleared.
7. The user can choose to draw another shape or exit the program.

---

### Triangle and Isosceles Trapezoid

Made by: José Fabián Molina Enríquez 

#### Introduction

This code is designed to allow the user to select one of two geometric shapes (a triangle or an isosceles trapezoid), specify an initial starting position for the turtle, and then have the turtle draw the selected shape within the `turtlesim` work area (assumed to be from 0 to 11 on both the X and Y axes). The program displays the coordinates of each vertex on the console and checks that all vertices lie within the work area. If any vertex is outside the area, the user is notified and prompted for a new starting position. When the drawing is complete, the program waits for the user to press a key before clearing the screen and repositioning the turtle back to the starting point.

A key requirement is the use of a proportional controller. In the version provided here, the turtle’s movement is split into two phases when moving to a vertex:

1. **Rotation Phase**: The turtle rotates in place until its orientation is aligned with the target vertex.
2. **Translation Phase**: Once properly oriented, the turtle moves straight toward the vertex.

This two-phase approach ensures that the turtle draws straight lines rather than curves.

#### 2. Dependencies and Libraries

The code uses several ROS libraries and message/service types:

- **`rospy`**: Provides the ROS Python API.
- **`geometry_msgs.msg.Twist`**: Used for sending velocity commands.
- **`turtlesim.msg.Pose`**: For receiving the current position and orientation of the turtle.
- **`turtlesim.srv.TeleportAbsolute`**: Allows the turtle to be teleported to a specific location.
- **`std_srvs.srv.Empty`**: Used to call the service that clears the `turtlesim` window.

These libraries allow the code to subscribe to the turtle’s pose, publish velocity commands, and interact with services provided by `turtlesim` (teleporting and clearing the screen).

#### 3. Code Structure and General Description

The code is organized into several sections:

### a) Global Variables

- **`current_pose`**: A global variable that stores the current pose (position and orientation) of the turtle. It is updated by a callback function.

##### b) Utility Functions

- **`pose_callback(msg)`**: Updates `current_pose` with the latest pose data from the `/turtle1/pose` topic.
- **`is_within_area(x, y, x_min=0, x_max=11, y_min=0, y_max=11)`**: Checks whether a given point `(x, y)` is within the defined work area.

##### c) ROS Service Functions

- **`clear_screen()`**: Calls the `/clear` service to clear the `turtlesim` window.
- **`teleport_to(x, y, theta=0)`**: Uses the `/turtle1/teleport_absolute` service to move the turtle to a specific location and orientation.

##### d) Shape Drawing Functions

- **`draw_triangle(initial_x, initial_y)`**: Calculates the vertices of an equilateral triangle (with preset dimensions) based on the initial position. It displays the vertices on the console, validates that all vertices are inside the work area, and then draws the triangle by moving the turtle from vertex to vertex.
- **`draw_trapezoid(initial_x, initial_y)`**: Calculates the vertices of an isosceles trapezoid, displays their coordinates, validates their positions, and draws the shape by sequentially moving to each vertex.

##### e) Main Function

- **`main()`**: Initializes the ROS node and subscribes to the turtle’s pose topic. It presents a menu for the user to select which shape to draw or to exit. It requests the starting position and, based on the user's choice, teleports the turtle to that position and calls the appropriate shape-drawing function. Once the drawing is finished, it waits for a key press, clears the screen, and returns the turtle to its initial position.


#### 4. Proportional Controller (Two-Phase Movement)

The core movement function, `move_to_point(target_x, target_y)`, uses a proportional controller split into two phases:

##### Phase 1: Orientation

- **Goal**: Rotate the turtle in place until its orientation aligns with the direction of the target vertex.
- **Implementation**:
  - Calculate the desired angle using `math.atan2`.
  - Compute the angular error (difference between desired angle and current orientation).
  - Normalize the angular error to ensure it lies within the range `[-π, π]`.
  - Apply a proportional control (e.g., `twist.angular.z = 2.0 * angle_error`) until the error is very small (below a set threshold).

##### Phase 2: Translation

- **Goal**: Move the turtle straight toward the target vertex since the orientation is already correct.
- **Implementation**:
  - Calculate the distance to the target.
  - Use a proportional controller for linear movement (e.g., `twist.linear.x = 1.0 * distance`) while allowing for slight angular correction if necessary.
  - Continue until the turtle is within a small threshold distance from the target.

This two-phase approach guarantees that the turtle draws straight segments by ensuring it first faces the correct direction before moving.

#### 5. Conclusions and Considerations

- **Modularity**: The code is divided into functions for specific tasks (pose handling, service calls, drawing shapes), making it easier to maintain and extend.
- **ROS and Turtlesim Interaction**: The code leverages ROS topics and services to control the turtle’s behavior, including teleporting, clearing the screen, and moving with proportional control.
- **Proportional Controller**: The proportional controller is used both for angular adjustment and for moving the turtle, split into orientation and translation phases to ensure straight-line movement.
- **Work Area Validation**: Before drawing, each vertex is validated to ensure it lies within the defined work area. If not, the user is informed to provide a new starting position.
