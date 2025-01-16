
_instances = {}

class Singleton(type):
    """
    Casserole Singleton Infrastructure
    Based on https://stackoverflow.com/q/6760685 - creating
    singletons with metaclasses. Namely, any class which should
    be a singleton should inherit `metaclass=Singleton` in its constructor
    On the first instantiaion, the single instance will be created and added
    to the global _instances dictionary
    """
    def __call__(cls, *args, **kwargs):
        if cls not in _instances:
            _instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return _instances[cls]


def destroyAllSingletonInstances():
    """
    For unit testing purposes, we will need to simulate the roboRIO power cycling
    One part of this is to get new instances of all the singletons constructed
    This should never be called (nor NEED to be called) on the real robot.
    """
    global _instances
    _instances = {}

def noSingletonsAround():
    return len(_instances)==0

class ShortSingltonLivesUnderTest:
    # https://stackoverflow.com/questions/26405380/how-do-i-correctly-setup-and-teardown-for-my-pytest-class-with-tests

    @classmethod
    def setup_class(cls): # pylint: disable=invalid-name
        assert noSingletonsAround()


    @classmethod
    def teardown_class(cls): # pylint: disable=invalid-name
        destroyAllSingletonInstances()
