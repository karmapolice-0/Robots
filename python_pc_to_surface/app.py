from utils.userinterface import UserInterface


class App:
    """The main class of the app."""

    def __init__(self):
        self.ui = UserInterface()

    def method1(self):
        print("Method 1: done")

    def method2(self):
        print("Method 2: done")

    def method3(self):
        print("Method 3: done")

    def app_menu(self):
        """The main menu of the app."""

        MAIN_MENU = {
            1: {
                "label": "Using centers and rotations.",
                "func": self.method1
            },
            2: {
                "label": "Using Iterative-Closest-Point algorithm.",
                "func": self.method2
            },
            3: {
                "label": "RANSAC.",
                "func": self.method3
            }
        }

        self.ui.choose_menu("############ ALIGNMENT_METHODS ###########", MAIN_MENU)


a = App()
a.app_menu()