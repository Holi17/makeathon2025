import moving as mvState
import state as st
import time
import move as mv

class Turn(st.State): # State which turns the robot if a collision happened (Transit to this if ultrasonic detects collision)
    def execute(self):
        print("Turn")
        self.robot.change_state(mvState.Moving(self.robot))
        try:

            mv.setup()
            runing = True
            while runing:
                mv.DriveBackward()
                mv.motorStop()
                time.sleep(2)

                mv.DriveLeft()
                time.sleep(mv.SPIN_TIME)
                mv.motorStop()

                mv.DriveRight()
                time.sleep(mv.SPIN_TIME)
                mv.motorStop()

                mv.DriveStraight()
                time.sleep(mv.FORWARD_SHORT_TIME)
                mv.motorStop()

        except KeyboardInterrupt:
            mv.destroy()
