from positionSchemes.new_plunge_v1 import PlungeV1
#rah rah rah

class Poser:
    def __init__(self):
        self.haha = 0
        self.plunge = PlungeV1()

    def update(self):
        # figure out what is being clicked and all that... determine what we're doing.
        # given what we're doing, set the variable to the class of the action
        self.plunge.update()
        pass

    def updateDriveTrain(self, curCmd):
        return curCmd #(or return the fun stuff from the schemer)

    def updateElevator(self):
        pass

    def updateArm(self):
        pass


