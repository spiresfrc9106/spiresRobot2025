# pylint: disable-all
from utils.signalLogging import addLog, logUpdate

class SimpleClass():

    def __init__(self):
        self._x = 0

    def setX(self, x):
        self._x = x

    def x(self):
        return self._x




def test_simple_addLog():

    source = SimpleClass()

    addLog("test_simple_addLog", source.x, "counts")

    logUpdate()

    source.setX(1)

    logUpdate()
