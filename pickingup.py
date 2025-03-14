import time
import storing as stState
import state as st

class PickingUp(st.State): 
    servo0_max = 600 # ToDo: Adjust
    servo0_min = 5

    servo1_max = 700 
    servo1_min = 20

    servo2_max = 700 
    servo2_min = 20

    servo3_max = 650 
    servo3_min = 10

    def execute(self):
        print("PickingUp")
        
        size = 10 # needed any value
        # while True:
            # size = ultra.get_trashsize()

        if size < 10: # ToDo: Adjust size values
            self.grab('small')
        elif size < 20:
            self.grab('medium')
        elif size < 30:
            self.grab('large')
        else:
            time.sleep(0.2) 
            print("Too large")   
            self.robot.change_state(stState.Turn(self.robot))
        
        self.robot.change_state(stState.Storing(self.robot))

    def grab(self, size):
        # Adjust arm position values depending on size
        if size == 'small':
            servo1_pos = 0 # needed any value
            # servo2_pos =
            # servo3_pos =
            # servo4_pos =
            # set arm position values for small trash
        elif size == 'medium':
            servo1_pos = 1 # needed any value
            # servo2_pos =
            # servo3_pos =
            # servo4_pos =
            # set arm position values for medium trash
        elif size == 'large':
            servo1_pos = 2 # needed any value
            # servo2_pos =
            # servo3_pos =
            # servo4_pos =
            # set arm position values for large trash
        else:
            return
        
        # Step 1: Rotate base to default forward
        self.move_servo() # ToDo: To position; which servo(s)
        time.sleep(0.3)

        # Step 2: Lower the arm
        self.move_servo() # ToDo: To position; which servo(s)
        time.sleep(0.5)

        # Step 3: Close gripper to grab object
        self.move_servo()  # ToDo: To position; which servo(s)
        time.sleep(0.5)

        # Step 4: Lift arm
        self.move_servo() # ToDo: To position; which servo(s)
        time.sleep(0.5)

    #def move_servo(pos, servo):
    def move_servo(self):
        servo = 0 # needed any value
        # control range if illegal
        if servo == 0:
            pos = self.ctrl_range(pos, self.servo0_max, self.servo0_min)
        elif servo == 1:
            pos = self.ctrl_range(pos, self.servo1_max, self.servo1_min)
        elif servo == 2:
            pos = self.ctrl_range(pos, self.servo2_max, self.servo2_min)
        elif servo == 3:
            pos = self.ctrl_range(pos, self.servo3_max, self.servo3_min)
        
        pwm.set_pwm(pos, servo)

    def ctrl_range(self, pos, max, min):
        # set servo to legal angle if illegal
        if pos > max:
            return int(max)
        elif pos < min:
            return int(min)
        else:
            return int(pos)