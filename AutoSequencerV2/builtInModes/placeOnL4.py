from AutoSequencerV2.builtInCommands.doNothingCommand import DoNothingCommand
from AutoSequencerV2.mode import Mode, DesiredAutonAction
from humanInterface.operatorInterface import OperatorInterface


# Made for placing on L4 during autonomous mode, set up so that in the future we have more flexibility if we want to do something else in autonomous rather than just place at L4
class PlaceOnL4(Mode):
    def __init__(self):
        Mode.__init__(self, f"Place On L4")
        Mode.setDesiredAutonAction(self, DesiredAutonAction.PLACE_ON_L4)


    def getCmdGroup(self):
        return DoNothingCommand()
