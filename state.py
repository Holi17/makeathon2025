class State: # Super class for defining different states
    def __init__(self, robot):
        self.robot = robot

    def execute(self):
        pass