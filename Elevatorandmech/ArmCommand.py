class ArmCommand:
    __slots__ = ('angleDeg', 'velocityDegps')

    def __init__(self, angleDeg, velocityDegps):
        self.angleDeg = angleDeg
        self.velocityDegps = velocityDegps