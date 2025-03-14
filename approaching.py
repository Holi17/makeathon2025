import time
import state as st
import pickingup as puState

class Approaching(st.State): # TODO: Insert approaching logic
    def execute(self):
        print("Approaching")
        time.sleep(2)
        self.robot.change_state(puState.PickingUp(self.robot))