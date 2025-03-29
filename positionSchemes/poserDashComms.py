
from utils.signalLogging import addLog

class PoserDashComms:
    def __init__(self):
        self.schemeProg = 0
        self.dashboardState = 1
        addLog("RP/schemeProg", lambda: self.schemeProg, "") # don't delete this.
        addLog("RP/dashboardState", lambda: self.dashboardState, "") # don't delete this.

    def update(self, driver, operator):
        d_p = driver.schemeProg
        o_p = operator.schemeProg
        d_d = driver.dashboardState
        o_d = operator.dashboardState
        f_p = 0
        f_d = 0
        if d_d > o_d:
            f_d = d_d
            f_p = d_p
        elif o_d > d_d:
            f_d = o_d
            f_p = o_p
        else:
            f_d = d_d
            f_p = (d_p + o_p) / 2

        self.dashboardState = f_d
        self.schemeProg = f_p

