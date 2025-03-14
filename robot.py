import time
import moving as mv

class Robot:
    def __init__(self):
        self.state = mv.Moving(self) # Transit to moving state after robot init

    def change_state(self, new_state):
        self.state = new_state

    def run(self): # Start the robot
        while True:
            self.state.execute()
            time.sleep(1)