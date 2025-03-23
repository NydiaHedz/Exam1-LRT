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

### Main 
Made by: Nydia Hernández Bravo

The project consists of a main controller (`MainController`) that initializes and manages different modules for drawing geometric figures. Each module is responsible for drawing a specific pair of figures. The user interacts with the system through a text-based menu to select which figures to draw.

#### Code Structure

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

## How It Works

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
