
class ElevatorCommand:
    __slots__ = ('heightIn', 'velocityInps')

    def __init__(self, heightIn, velocityInps):
        self.heightIn = heightIn
        self.velocityInps = velocityInps