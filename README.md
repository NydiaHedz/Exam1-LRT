# Exam1-LRT

This repository contains two exercises implemented using ROS (Robot Operating System) to control turtles in the turtlesim simulator.

## Exercise 1: Random Turtle Spawning
This program generates 5 additional turtles at random positions within the Turtlesim workspace. Each turtle is placed at random (x,y) coordinates with a random orientation. The program uses Turtlesim's Spawn service to create the new instances.

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

