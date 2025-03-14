import moving as mvState
import state as st

class Turn(st.State): # State which turns the robot if a collision happened (Transit to this if ultrasonic detects collision)
    def execute(self):
        print("Turn")
        self.robot.change_state(mvState.Moving(self.robot))