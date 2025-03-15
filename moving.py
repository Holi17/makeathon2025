import state as st
import move as mv
import time
import approaching as apState
class Moving(st.State): # TODO: Insert moving logic
    def execute(self):
        print("Moving")
        time.sleep(2)
        self.robot.change_state(apState.Approaching(self.robot))
         try:
             speed_set = 60
             mv.setup()
             running = True
             while running:
                 mv.DriveStraight()
                 time.sleep(mv.FORWARD_LONG_TIME)
                 mv.motorStop()

                 mv.DriveRight()
                 time.sleep(mv.SPIN_TIME)
                 mv.motorStop()

                 mv.DriveStraight()
                 time.sleep(mv.FORWARD_SHORT_TIME)
                 mv.motorStop()

                 mv.DriveRight()
                 time.sleep(mv.SPIN_TIME)
                 mv.motorStop()

                 mv.DriveStraight()
                 time.sleep(mv.FORWARD_LONG_TIME)
                 mv.motorStop()

                 mv.DriveLeft()
                 time.sleep(mv.SPIN_TIME)
                 mv.motorStop()

                 mv.DriveStraight()
                 time.sleep(mv.FORWARD_SHORT_TIME)
                 mv.motorStop()

                 mv.DriveLeft()
                 time.sleep(mv.SPIN_TIME)
                 mv.motorStop()

         except KeyboardInterrupt:
	    #     mv.destroy()
