#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
import sys
import time
import random  # For generating random positions

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)

    # Wait until the /spawn and /kill services are available
    rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/kill')

    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        kill_turtle = rospy.ServiceProxy('/kill', Kill)

        # Attempt to delete only the turtle named "turtle1"
        try:
            kill_turtle('turtle1')  # Delete the first turtle if it exists
            print("Turtle 1 deleted.")
        except rospy.ServiceException:
            print("Turtle 1 does not exist, cannot delete.")

        # Create 5 turtles at random positions within the turtlesim space (0.0 to 11.0)
        for i in range(5):
            # Generate a random position for x and y within the range 0.0 to 11.0
            x = random.uniform(1.0, 10.0)  # Generate a random value between 0.0 and 11.0 for x
            y = random.uniform(1.0, 10.0)  # Generate a random value between 0.0 and 11.0 for y
            
            # Spawn the turtle with random coordinates
            spawn_turtle(x, y, 0.0, f'turtle{i+1}')  # Random coordinates and unique name
            print(f"Generated turtle {i+1} at position ({x:.2f}, {y:.2f}).")

    except rospy.ServiceException as e:
        print(f"Error spawning turtles: {e}")

    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    main()
