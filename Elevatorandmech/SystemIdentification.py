from wpilib import XboxController

class SystemIdentification:
    def __init__(self):
        ctrlIdx = 0
        self.ctrl = XboxController(ctrlIdx)
        pass

    def update(self):
        #Detect if RB is being held down, if so, then the elevator will be controlled with Raw Joystick Input, which uses velocity
        #If RB is not being held down, then certain buttons will be set to pre-set positions that we need, ex. pressing x will put the robot through a combination of movements to get ready for picking up coral from the coral station
        if self.ctrl.getRightBumper():
            #RAW JOYSTICK INPUT
            print("Right Bumper Pressed")
            pass
        else:
            print("Right Bumper Not Pressed")
            #PRE-SET POSITION BUTTONS
            pass

