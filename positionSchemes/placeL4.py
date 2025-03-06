import numpy as np

class placeL4Scheme:
    def __init__(self):
        # in this scheme, the goal is to place on the top one.
        # we are going to control the base, elevator, and arm


        # keep these? probably not.
        # base f/b is f/d distance from april tag
        # base l/r is l/r distance from april tag
        # elevator is proportion of elevator raised
        # arm is proportion of a half circle with 0 at bottom, 1 at top up

        self.time = np.array([
            0.00,
        ])
        #       position xyt            velocity xyt
        self.base = np.array([
            [[0.000, 0.000, 0.000], [0.000, 0.000, 0.000]],
        ])
        #
        self.elevator = np.array([
            [0.000, 0.000]
        ])
        self.arm = np.array([
            [0.000, 0.000]
        ])


    def updateBase(self):
        #example call: path[t, 0]
