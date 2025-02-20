#the max speed/acceleration the arm can go and its gearbox options
from utils.calibration import Calibration
from utils.units import in2m

#All Numbers are placeholders for now (numbers used last year)
setup = None

if setup == "modifiedElev":
    ARM_GEARBOX_GEAR_RATIO = 5.0/1.0
elif setup == "robotArm":
    ARM_GEARBOX_GEAR_RATIO = 50.0/1.0
else:
    ARM_GEARBOX_GEAR_RATIO = 5.0/1.0
MAX_ARM_VEL_DPS = 10 # degrees per second
MAX_ARM_ACCEL_DPS2 = 5 # degrees per second per second


