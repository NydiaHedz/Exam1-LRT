#!/usr/bin/env python3

import rospy
from cuadrado_romboide import CuadradoRomboide
from rombo_pentagono import RomboPentagono
from triangulo_trapecio import TrianguloTrapecio
from rectangulo_trapezoide import RectanguloTrapezoide

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

if __name__ == '__main__':
    try:
        controller = MainController()
        controller.menu()
    except rospy.ROSInterruptException:
        pass
