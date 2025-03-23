#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from std_srvs.srv import Empty
import time
import math

class RectanguloTrapezoide:
    def __init__(self):
        # Initialize variables
        self.pub = None
        self.x_actual = 0
        self.y_actual = 0
        self.rate = rospy.Rate(10)  # 10 Hz frequency

        # Workspace dimensions
        self.ancho_area = 11
        self.alto_area = 11

        # Kill the initial turtle at (5.5, 5.5)
        self.kill_initial_turtle()

    def actualizar_posicion(self, pose):
        """Callback to get the current position of the turtle."""
        self.x_actual = pose.x
        self.y_actual = pose.y

    def suscribirse_posicion(self, nombre_tortuga):
        """Subscribe to the turtle's position topic."""
        rospy.Subscriber(f'/{nombre_tortuga}/pose', Pose, self.actualizar_posicion)

    def configurar_publicador(self, nombre_tortuga):
        """Configure the velocity publisher for the turtle."""
        self.pub = rospy.Publisher(f'/{nombre_tortuga}/cmd_vel', Twist, queue_size=10)

    def kill_initial_turtle(self):
        """Kill the default turtle in the workspace."""
        rospy.wait_for_service('/kill')
        try:
            kill = rospy.ServiceProxy('/kill', Kill)
            kill("turtle1")  # Kill the default turtle named 'turtle1'
            rospy.loginfo("Initial turtle 'turtle1' killed.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to kill initial turtle: {e}")

    def crear_tortuga(self, x, y, nombre):
        """Create a turtle at the given coordinates."""
        rospy.wait_for_service('/spawn')
        try:
            spawn = rospy.ServiceProxy('/spawn', Spawn)
            spawn(x, y, 0.0, nombre)
            rospy.loginfo(f"Turtle '{nombre}' created at ({x}, {y}).")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to create turtle '{nombre}': {e}")

    def mover_a_punto(self, x_destino, y_destino):
        """Move the turtle to a point using proportional control."""
        Kp = 1.0

        while not rospy.is_shutdown():
            error_x = x_destino - self.x_actual
            error_y = y_destino - self.y_actual

            if abs(error_x) < 0.05 and abs(error_y) < 0.05:
                rospy.loginfo(f"Reached point: ({x_destino}, {y_destino})")
                break

            vel_x = max(min(Kp * error_x, 2.0), -2.0)
            vel_y = max(min(Kp * error_y, 2.0), -2.0)

            msg = Twist()
            msg.linear.x = vel_x
            msg.linear.y = vel_y

            self.pub.publish(msg)
            self.rate.sleep()

    def validar_espacio(self, puntos):
        """Check if the points fit within the workspace."""
        for x, y in puntos:
            if x < 0 or x > self.ancho_area or y < 0 or y > self.alto_area:
                return False
        return True

    def dibujar_rectangulo(self, x_inicio, y_inicio):
        """Draw a rectangle."""
        rospy.loginfo("Drawing a rectangle...")

        ancho = 3.0
        alto = 2.0

        puntos = [
            (x_inicio, y_inicio),
            (x_inicio + ancho, y_inicio),
            (x_inicio + ancho, y_inicio + alto),
            (x_inicio, y_inicio + alto),
            (x_inicio, y_inicio)  # Return to the starting point to close the rectangle
        ]

        if not self.validar_espacio(puntos):
            print("The rectangle does not fit in the workspace. Try different coordinates.")
            return False

        for x, y in puntos:
            self.mover_a_punto(x, y)
            time.sleep(1)

        # Notify the user that the rectangle has been drawn
        print("Rectangle drawn successfully!")
        input("Press any key to return to the menu...")

        self.matar_tortuga("turtle2")  # Kill the turtle after drawing the rectangle
        self.limpiar_area()  # Clear the workspace after drawing

        return True

    def dibujar_trapezoide(self, x_inicio, y_inicio):
        """Draw a trapezoid."""
        rospy.loginfo("Drawing a trapezoid...")

        base1 = 4.0
        base2 = 2.0
        altura = 3.0

        puntos = [
            (x_inicio, y_inicio),
            (x_inicio + base1, y_inicio),
            (x_inicio + base2, y_inicio + altura),
            (x_inicio, y_inicio + altura),
            (x_inicio, y_inicio)  # Return to the starting point to close the trapezoid
        ]

        if not self.validar_espacio(puntos):
            print("The trapezoid does not fit in the workspace. Try different coordinates.")
            return False

        for x, y in puntos:
            self.mover_a_punto(x, y)
            time.sleep(1)

        # Notify the user that the trapezoid has been drawn
        print("Trapezoid drawn successfully!")
        input("Press any key to return to the menu...")

        self.matar_tortuga("turtle2")  # Kill the turtle after drawing the trapezoid
        self.limpiar_area()  # Clear the workspace after drawing

        return True

    def matar_tortuga(self, nombre_tortuga):
        """Kill the turtle after drawing the figure."""
        rospy.wait_for_service('/kill')
        try:
            kill = rospy.ServiceProxy('/kill', Kill)
            kill(nombre_tortuga)
            rospy.loginfo(f"Turtle '{nombre_tortuga}' killed.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to kill turtle '{nombre_tortuga}': {e}")

    def limpiar_area(self):
        """Clear everything drawn in the workspace."""
        rospy.wait_for_service('/clear')
        try:
            clear = rospy.ServiceProxy('/clear', Empty)
            clear()
            rospy.loginfo("Workspace cleared.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to clear the workspace: {e}")

    def menu(self):
        """Menu for drawing a rectangle or trapezoid."""
        while not rospy.is_shutdown():
            print("\nRectangle and Trapezoid Menu")
            print("1 -> Draw Rectangle")
            print("2 -> Draw Trapezoid")
            print("3 -> Return to Main Menu")

            opcion = input("Select an option: ")

            if opcion == '3':
                print("Returning to main menu...")
                break

            elif opcion == '1':
                print("Selected option: Draw Rectangle")

                while True:
                    print("\nCoordinates for the rectangle")
                    x_rectangulo = float(input("X coordinate: "))
                    y_rectangulo = float(input("Y coordinate: "))

                    self.crear_tortuga(x_rectangulo, y_rectangulo, 'turtle2')  # Create turtle at the provided coordinates
                    self.suscribirse_posicion('turtle2')
                    self.configurar_publicador('turtle2')

                    if not self.dibujar_rectangulo(x_rectangulo, y_rectangulo):
                        continue
                    break

            elif opcion == '2':
                print("Selected option: Draw Trapezoid")

                while True:
                    print("\nCoordinates for the trapezoid")
                    x_trapezoide = float(input("X coordinate: "))
                    y_trapezoide = float(input("Y coordinate: "))

                    self.crear_tortuga(x_trapezoide, y_trapezoide, 'turtle2')  # Create turtle at the provided coordinates
                    self.suscribirse_posicion('turtle2')
                    self.configurar_publicador('turtle2')

                    if not self.dibujar_trapezoide(x_trapezoide, y_trapezoide):
                        continue
                    break
