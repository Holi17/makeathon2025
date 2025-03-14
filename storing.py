import time
import state as st
import moving as mvState

class Storing(st.State): # TODO: Insert moving logic
    def execute(self):
        print("Storing")
        time.sleep(2)
        self.robot.change_state(mvState.Moving(self.robot))