from AutoSequencerV2.command import Command


class DoNothingCommand(Command):
    def isDone(self):
        return False

    def getName(self):
        return f"Place At L4"
