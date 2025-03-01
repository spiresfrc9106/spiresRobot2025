#the max speed/acceleration the arm can go and its gearbox options
from drivetrain.robotDependentConstants import RobotDependentConstants
from utils.calibration import Calibration
from utils.robotIdentification import RobotIdentification
from utils.units import in2m

#All Numbers are placeholders for now (numbers used last year)

robotDepConstants = RobotDependentConstants().get()[RobotIdentification().getRobotType()]

if robotDepConstants['HAS_ARM']:
    ARM_GEARBOX_GEAR_RATIO = robotDepConstants['ARM_GEARBOX_GEAR_RATIO']
else:
    ARM_GEARBOX_GEAR_RATIO = 5.0/1.0
MAX_ARM_VEL_DPS = 10 # degrees per second
MAX_ARM_ACCEL_DPS2 = 5 # degrees per second per second


