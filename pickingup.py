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
        
        while True:
            size = ultra.get_trashsize()

        if size < 10: # ToDo: Adjust size values
                grab('small')
            elif size < 20:
                grab('medium')
            elif size < 30:
                grab('large')
            else:
                time.sleep(0.2) 
                print("Too large")   
                self.robot.change_state(stState.Turn(self.robot))
        
        self.robot.change_state(stState.Storing(self.robot))

    def grab(size):
        # Adjust arm position values depending on size
        if size == 'small':
            # servo1_pos =
            # servo2_pos =
            # servo3_pos =
            # servo4_pos =
            # set arm position values for small trash
        elif size == 'medium':
            # servo1_pos =
            # servo2_pos =
            # servo3_pos =
            # servo4_pos =
            # set arm position values for medium trash
        elif size == 'large':
            # servo1_pos =
            # servo2_pos =
            # servo3_pos =
            # servo4_pos =
            # set arm position values for large trash
        else:
            return
        
        # Step 1: Rotate base to default forward
        move_servo() # ToDo: To position; which servo(s)
        time.sleep(0.3)

        # Step 2: Lower the arm
        move_servo() # ToDo: To position; which servo(s)
        time.sleep(0.5)

        # Step 3: Close gripper to grab object
        move_servo()  # ToDo: To position; which servo(s)
        time.sleep(0.5)

        # Step 4: Lift arm
        move_servo() # ToDo: To position; which servo(s)
        time.sleep(0.5)

    #def move_servo(pos, servo):
    def move_servo():
        # control range if illegal
        if servo == 0:
            pos = ctrl_range(pos, servo0_max, servo0_min)
        elif servo == 1:
            pos = ctrl_range(pos, servo1_max, servo1_min)
        elif servo == 2:
            pos = ctrl_range(pos, servo2_max, servo2_min)
        elif servo == 3:
            pos = ctrl_range(pos, servo3_max, servo3_min)
        
        pwm.set_pwm(pos, servo)

    def ctrl_range(pos, max, min):
        # set servo to legal angle if illegal
        if pos > max:
            return int(max)
        elif pos < min:
            return int(min)
        else:
            return int(pos)