import ntcore as nt
from AutoSequencerV2.mode import Mode


# A mode list is the set of autonomous modes that the drive team must pick from before a match
# Networktables is used to read the user's current selection
class ModeList:
    def __init__(self, name):
        self.modes = []
        self._name = name
        self.curModeIdx = 0  # default

        inst = nt.NetworkTableInstance.getDefault()

        curModeIdxTopic = inst.getIntegerTopic(self.getCurModeTopicName())
        self.curModeIdxPublisher = curModeIdxTopic.publish(
            nt.PubSubOptions(sendAll=False, keepDuplicates=False)
        )
        self.curModeIdxPublisher.setDefault(self.curModeIdx)

        desModeIdxTopic = inst.getIntegerTopic(self.getDesModeTopicName())
        self.desModeIdxPublisher = desModeIdxTopic.subscribe(self.curModeIdx)

    def addMode(self, modeIn):
        self.modes.append(modeIn)

    def deleteOppositeColorModes(self):
        # todo Noah Change this set-in-stone loop to make it more flexible with Main Mode List changes, deleting modes based on the Name rather than starting at index 2
        for i in range(1, len(self.modes)):
            del self.modes[-1]

        self.curModeIdx = 0


    def updateMode(self, force=False):
        prevModeIdx = self.curModeIdx
        tmp = self.desModeIdxPublisher.getAtomic()
        if tmp.time > 0 or force:
            self.curModeIdx = tmp.value
            self.curModeIdxPublisher.set(self.curModeIdx)

        return (
                prevModeIdx != self.curModeIdx
        )  # Return true if the selection has changed

    def getCurMode(self) -> Mode:
        try:
            return self.modes[self.curModeIdx]
        except:
            return self.modes[0]

    def getNames(self):
        return [x.getName() for x in self.modes]

    def getDesModeTopicName(self):
        return self.getModeTopicBase() + "/des"

    def getCurModeTopicName(self):
        return self.getModeTopicBase() + "/cur"

    def getModeTopicBase(self):
        return f"/Autonomous/{self._name}"
