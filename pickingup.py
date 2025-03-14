import time
import storing as stState
import state as st

class PickingUp(st.State): # TODO: Insert picking up logic
    def execute(self):
        print("PickingUp")
        time.sleep(2)
        self.robot.change_state(stState.Storing(self.robot))